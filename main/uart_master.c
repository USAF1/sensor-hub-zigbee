/*
 * uart_master.c
 * Sensor Hub <-> Master UART Communication Layer
 * Innovatsii EMS — Pico 1
 *
 * Outbound path:
 *   Caller → uart_master_send_*() → tx_send_fmt() → tx_enqueue()
 *   → s_tx_queue → uart_tx_task() → uart_write_bytes()
 *
 * Inbound path:
 *   uart_rx_task() → accumulate into s_rx_line[]
 *   → on '\n': validate '{', dispatch_command()
 *   → k_cmd_table[] → handler() → uart_hooks.c
 *
 * JSON parser:
 *   Handles both "key":"value" and "key": "value" (MicroPython spaced form).
 *   Skips optional whitespace after every colon before reading value.
 *
 * Memory:
 *   Zero heap allocation after init.
 *   All queues, stacks, buffers use static storage.
 */

#include "uart_master.h"
#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char *TAG = "UART_M";

// ============================================================================
// INTERNAL TYPES
// ============================================================================

typedef struct {
    char     data[UART_MASTER_TX_MSG_SIZE];
    uint16_t len;
} tx_msg_t;

typedef struct {
    const char *type_str;
    void (*handler)(const char *json, uint16_t len);
} cmd_entry_t;

// ============================================================================
// STATIC STORAGE
// ============================================================================

static QueueHandle_t    s_tx_queue = NULL;
static StaticQueue_t    s_tx_queue_cb;
static uint8_t          s_tx_queue_buf[UART_MASTER_TX_QUEUE_DEPTH * sizeof(tx_msg_t)];

static StackType_t  s_tx_stack[UART_MASTER_TX_TASK_STACK];
static StaticTask_t s_tx_tcb;
static StackType_t  s_rx_stack[UART_MASTER_RX_TASK_STACK];
static StaticTask_t s_rx_tcb;
static StackType_t  s_tmr_stack[UART_MASTER_TMR_TASK_STACK];
static StaticTask_t s_tmr_tcb;

static char     s_rx_line[UART_MASTER_LINE_BUF_SIZE];
static uint16_t s_rx_pos = 0;

static SemaphoreHandle_t s_cfg_mutex = NULL;
static StaticSemaphore_t s_cfg_mutex_cb;
static uart_hub_config_t s_config;

typedef struct {
    bool     door_open;
    bool     alarm_sent;
    uint32_t open_since_sec;
} door_alarm_t;
static door_alarm_t s_door_alarm[MAX_SENSORS];

static uint32_t s_last_heartbeat_sec = 0;

// ============================================================================
// UPTIME HELPER
// ============================================================================

static void uptime_str(char *buf, size_t len)
{
    uint64_t sec = esp_timer_get_time() / 1000000ULL;
    snprintf(buf, len, "%02u:%02u:%02u",
             (unsigned)(sec / 3600),
             (unsigned)((sec % 3600) / 60),
             (unsigned)(sec % 60));
}

// ============================================================================
// DEFAULT CONFIG AND VALIDATION
// ============================================================================

static const uart_hub_config_t k_defaults = {
    .pairing_duration_sec      = 120,
    .watchdog_enable           = true,
    .watchdog_interval_min     = 60,
    .watchdog_ping_timeout_sec = 30,
    .door_alarm_threshold_min  = 10,
    .heartbeat_interval_min    = 30,
};

static void config_clamp(uart_hub_config_t *c)
{
    if (c->pairing_duration_sec      < 30  ) c->pairing_duration_sec      = 30;
    if (c->pairing_duration_sec      > 600 ) c->pairing_duration_sec      = 600;
    if (c->watchdog_interval_min     < 1   ) c->watchdog_interval_min     = 1;
    if (c->watchdog_interval_min     > 1440) c->watchdog_interval_min     = 1440;
    if (c->watchdog_ping_timeout_sec < 10  ) c->watchdog_ping_timeout_sec = 10;
    if (c->watchdog_ping_timeout_sec > 120 ) c->watchdog_ping_timeout_sec = 120;
    if (c->door_alarm_threshold_min  < 1   ) c->door_alarm_threshold_min  = 1;
    if (c->door_alarm_threshold_min  > 60  ) c->door_alarm_threshold_min  = 60;
    if (c->heartbeat_interval_min    < 1   ) c->heartbeat_interval_min    = 1;
    if (c->heartbeat_interval_min    > 1440) c->heartbeat_interval_min    = 1440;
}

// ============================================================================
// NVS CONFIG
// ============================================================================

#define NVS_NS  "uart_hub"
#define NVS_KEY "cfg_v1"

esp_err_t uart_master_load_config(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No UART config in NVS — using defaults");
        s_config = k_defaults;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed: %s — using defaults", esp_err_to_name(err));
        s_config = k_defaults;
        return err;
    }
    size_t sz = sizeof(uart_hub_config_t);
    err = nvs_get_blob(h, NVS_KEY, &s_config, &sz);
    nvs_close(h);
    if (err != ESP_OK || sz != sizeof(uart_hub_config_t)) {
        ESP_LOGW(TAG, "UART config blob invalid — using defaults");
        s_config = k_defaults;
        return ESP_OK;
    }
    config_clamp(&s_config);
    ESP_LOGI(TAG, "UART config loaded from NVS");
    return ESP_OK;
}

esp_err_t uart_master_save_config(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open for write failed: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_blob(h, NVS_KEY, &s_config, sizeof(uart_hub_config_t));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "UART config save failed: %s", esp_err_to_name(err));
    return err;
}

void uart_master_get_config(uart_hub_config_t *out)
{
    if (!out) return;
    if (xSemaphoreTake(s_cfg_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        *out = s_config;
        xSemaphoreGive(s_cfg_mutex);
    } else {
        ESP_LOGW(TAG, "get_config: mutex timeout — returning defaults");
        *out = k_defaults;
    }
}

esp_err_t uart_master_set_config(const uart_hub_config_t *in)
{
    if (!in) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_cfg_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "set_config: mutex timeout");
        return ESP_ERR_TIMEOUT;
    }
    s_config = *in;
    config_clamp(&s_config);
    xSemaphoreGive(s_cfg_mutex);
    return uart_master_save_config();
}

// ============================================================================
// TX QUEUE AND SEND HELPERS
// ============================================================================

static void tx_enqueue(const char *data, uint16_t len)
{
    if (!s_tx_queue || len == 0 || len >= UART_MASTER_TX_MSG_SIZE) {
        if (len >= UART_MASTER_TX_MSG_SIZE)
            ESP_LOGW(TAG, "tx_enqueue: msg too long (%u)", len);
        return;
    }
    tx_msg_t msg;
    memcpy(msg.data, data, len);
    msg.data[len] = '\0';
    msg.len = len;
    if (xQueueSend(s_tx_queue, &msg, 0) != pdTRUE)
        ESP_LOGW(TAG, "TX queue full — message dropped");
}

static void tx_send_fmt(const char *fmt, ...)
{
    char buf[UART_MASTER_TX_MSG_SIZE];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf) - 2, fmt, ap);
    va_end(ap);
    if (n <= 0 || n >= (int)(sizeof(buf) - 2)) {
        ESP_LOGW(TAG, "tx_send_fmt: truncation or error (n=%d)", n);
        return;
    }
    buf[n]     = '\n';
    buf[n + 1] = '\0';
    tx_enqueue(buf, (uint16_t)(n + 1));
}

// ============================================================================
// TX TASK
// ============================================================================

static void uart_tx_task(void *arg)
{
    (void)arg;
    tx_msg_t msg;
    for (;;) {
        if (xQueueReceive(s_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.len > 0 && msg.len < UART_MASTER_TX_MSG_SIZE)
                uart_write_bytes(UART_MASTER_PORT, msg.data, msg.len);
        }
    }
}

// ============================================================================
// JSON FIELD EXTRACTORS
// Handles both compact "key":"value" and spaced "key": "value" forms.
// MicroPython ujson.dumps() always produces spaced form.
// Both forms are handled by skipping whitespace after the colon.
// ============================================================================

static bool json_str(const char *json, const char *key,
                     char *out, size_t out_sz)
{
    char pat[64];
    int n = snprintf(pat, sizeof(pat), "\"%s\":", key);
    if (n <= 0 || n >= (int)sizeof(pat)) return false;

    const char *p = strstr(json, pat);
    if (!p) return false;
    p += strlen(pat);

    /* Skip optional whitespace after colon — MicroPython adds a space */
    while (*p == ' ' || *p == '\t') p++;

    if (*p != '"') return false;
    p++;  /* skip opening quote */

    const char *e = strchr(p, '"');
    if (!e) return false;

    size_t len = (size_t)(e - p);
    if (len >= out_sz) return false;

    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static bool json_int(const char *json, const char *key, int32_t *out)
{
    char pat[64];
    int n = snprintf(pat, sizeof(pat), "\"%s\":", key);
    if (n <= 0 || n >= (int)sizeof(pat)) return false;

    const char *p = strstr(json, pat);
    if (!p) return false;
    p += strlen(pat);

    /* Skip optional whitespace */
    while (*p == ' ' || *p == '\t') p++;

    char *ep;
    long v = strtol(p, &ep, 10);
    if (ep == p) return false;

    *out = (int32_t)v;
    return true;
}

static bool json_bool(const char *json, const char *key, bool *out)
{
    char pat[64];
    int n = snprintf(pat, sizeof(pat), "\"%s\":", key);
    if (n <= 0 || n >= (int)sizeof(pat)) return false;

    const char *p = strstr(json, pat);
    if (!p) return false;
    p += strlen(pat);

    /* Skip optional whitespace */
    while (*p == ' ' || *p == '\t') p++;

    if (strncmp(p, "true",  4) == 0) { *out = true;  return true; }
    if (strncmp(p, "false", 5) == 0) { *out = false; return true; }
    return false;
}

// ============================================================================
// COMMAND HANDLERS (inbound from Master)
// ============================================================================

static void cmd_set_sensor_name(const char *json, uint16_t len);
static void cmd_set_config     (const char *json, uint16_t len);
static void cmd_get_config     (const char *json, uint16_t len);
static void cmd_get_logs       (const char *json, uint16_t len);
static void cmd_start_pairing  (const char *json, uint16_t len);
static void cmd_stop_pairing   (const char *json, uint16_t len);
static void cmd_remove_sensor  (const char *json, uint16_t len);
static void cmd_factory_reset  (const char *json, uint16_t len);
static void cmd_restart        (const char *json, uint16_t len);

static const cmd_entry_t k_cmds[] = {
    { "set_sensor_name", cmd_set_sensor_name },
    { "set_config",      cmd_set_config      },
    { "get_config",      cmd_get_config      },
    { "get_logs",        cmd_get_logs        },
    { "start_pairing",   cmd_start_pairing   },
    { "stop_pairing",    cmd_stop_pairing    },
    { "remove_sensor",   cmd_remove_sensor   },
    { "factory_reset",   cmd_factory_reset   },
    { "restart",         cmd_restart         },
};
static const size_t k_cmds_len = sizeof(k_cmds) / sizeof(k_cmds[0]);

static void dispatch_command(const char *json, uint16_t len)
{
    char type[32];
    if (!json_str(json, "type", type, sizeof(type))) {
        ESP_LOGW(TAG, "RX: missing 'type' — discarding: %.60s", json);
        return;
    }
    for (size_t i = 0; i < k_cmds_len; i++) {
        if (strcmp(type, k_cmds[i].type_str) == 0) {
            k_cmds[i].handler(json, len);
            return;
        }
    }
    ESP_LOGW(TAG, "RX: unknown type '%s'", type);
}

static void cmd_set_sensor_name(const char *json, uint16_t len)
{
    (void)len;
    int32_t idx = -1;
    char    name[SENSOR_NAME_LEN];

    if (!json_int(json, "sensor_index", &idx) ||
        idx < 0 || idx >= MAX_SENSORS) {
        ESP_LOGW(TAG, "set_sensor_name: bad index");
        uart_master_send_ack("set_sensor_name", false);
        return;
    }
    if (!json_str(json, "name", name, sizeof(name))) {
        ESP_LOGW(TAG, "set_sensor_name: missing name");
        uart_master_send_ack("set_sensor_name", false);
        return;
    }

    hub_config_t *c = lock_config();
    if (!c) { uart_master_send_ack("set_sensor_name", false); return; }

    if ((int)idx >= c->sensor_count) {
        unlock_config();
        ESP_LOGW(TAG, "set_sensor_name: index %ld out of range", (long)idx);
        uart_master_send_ack("set_sensor_name", false);
        return;
    }
    strncpy(c->sensors[idx].sensor_name, name, SENSOR_NAME_LEN - 1);
    c->sensors[idx].sensor_name[SENSOR_NAME_LEN - 1] = '\0';
    unlock_config();
    mark_dirty();

    ESP_LOGI(TAG, "Sensor %ld renamed to '%s'", (long)idx, name);
    uart_master_send_ack("set_sensor_name", true);
}

static void cmd_set_config(const char *json, uint16_t len)
{
    (void)len;
    uart_hub_config_t nc;
    uart_master_get_config(&nc);

    int32_t tmp; bool btmp;
    if (json_int(json,  "pairing_duration_sec",      &tmp))  nc.pairing_duration_sec      = (uint16_t)tmp;
    if (json_bool(json, "watchdog_enable",            &btmp)) nc.watchdog_enable           = btmp;
    if (json_int(json,  "watchdog_interval_min",      &tmp))  nc.watchdog_interval_min     = (uint16_t)tmp;
    if (json_int(json,  "watchdog_ping_timeout_sec",  &tmp))  nc.watchdog_ping_timeout_sec = (uint16_t)tmp;
    if (json_int(json,  "door_alarm_threshold_min",   &tmp))  nc.door_alarm_threshold_min  = (uint16_t)tmp;
    if (json_int(json,  "heartbeat_interval_min",     &tmp))  nc.heartbeat_interval_min    = (uint16_t)tmp;

    esp_err_t err = uart_master_set_config(&nc);
    uart_master_send_ack("set_config", err == ESP_OK);
    if (err == ESP_OK) ESP_LOGI(TAG, "Config updated from Master");
}

static void cmd_get_config(const char *json, uint16_t len)
{
    (void)json; (void)len;
    uart_master_send_config_response();
}

static void cmd_get_logs(const char *json, uint16_t len)
{
    (void)json; (void)len;
    uart_master_send_log_response("Log ring buffer — Phase 8");
    uart_master_send_ack("get_logs", true);
}

static void cmd_start_pairing(const char *json, uint16_t len)
{
    (void)len;
    int32_t dur = -1;
    (void)json_int(json, "duration_sec", &dur);

    uart_hub_config_t cfg;
    uart_master_get_config(&cfg);
    uint16_t d = (dur > 0 && dur <= 600) ? (uint16_t)dur : cfg.pairing_duration_sec;

    extern void uart_cmd_start_pairing(uint16_t duration_sec);
    uart_cmd_start_pairing(d);
    uart_master_send_ack("start_pairing", true);
    ESP_LOGI(TAG, "Pairing opened for %us", (unsigned)d);
}

static void cmd_stop_pairing(const char *json, uint16_t len)
{
    (void)json; (void)len;
    extern void uart_cmd_stop_pairing(void);
    uart_cmd_stop_pairing();
    uart_master_send_ack("stop_pairing", true);
}

static void cmd_remove_sensor(const char *json, uint16_t len)
{
    (void)len;
    int32_t idx = -1;
    if (!json_int(json, "sensor_index", &idx) ||
        idx < 0 || idx >= MAX_SENSORS) {
        ESP_LOGW(TAG, "remove_sensor: bad index");
        uart_master_send_ack("remove_sensor", false);
        return;
    }
    extern void uart_cmd_remove_sensor(int idx);
    uart_cmd_remove_sensor((int)idx);
    uart_master_send_ack("remove_sensor", true);
}

static void cmd_factory_reset(const char *json, uint16_t len)
{
    (void)json; (void)len;
    ESP_LOGW(TAG, "Factory reset from Master");
    uart_master_send_ack("factory_reset", true);
    vTaskDelay(pdMS_TO_TICKS(200));
    extern void uart_cmd_factory_reset(void);
    uart_cmd_factory_reset();
}

static void cmd_restart(const char *json, uint16_t len)
{
    (void)json; (void)len;
    ESP_LOGW(TAG, "Restart from Master");
    uart_master_send_ack("restart", true);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

// ============================================================================
// RX TASK
// Accumulates bytes into static line buffer.
// Validates line starts with '{' before dispatching — filters binary noise.
// Non-printable bytes reset the accumulation buffer silently.
// ============================================================================

static void uart_rx_task(void *arg)
{
    (void)arg;
    uint8_t chunk[128];
    for (;;) {
        int bytes = uart_read_bytes(UART_MASTER_PORT, chunk,
                                    sizeof(chunk), pdMS_TO_TICKS(20));
        for (int i = 0; i < bytes; i++) {
            uint8_t b = chunk[i];

            if (b == '\r') continue;

            if (b == '\n') {
                if (s_rx_pos > 0) {
                    s_rx_line[s_rx_pos] = '\0';
                    if (s_rx_line[0] == '{') {
                        dispatch_command(s_rx_line, s_rx_pos);
                    } else {
                        ESP_LOGW(TAG, "RX: non-JSON discarded (starts 0x%02x)",
                                 (uint8_t)s_rx_line[0]);
                    }
                    s_rx_pos = 0;
                }
                continue;
            }

            /*
             * Drop non-printable bytes that cannot appear in JSON.
             * This filters binary noise from Zigbee radio interference
             * on the UART RX pin and resets any partial accumulation.
             */
            if (b < 0x20 && b != '\t') {
                if (s_rx_pos > 0) {
                    ESP_LOGW(TAG, "RX: non-printable 0x%02x — line reset", b);
                    s_rx_pos = 0;
                }
                continue;
            }

            if (s_rx_pos >= UART_MASTER_LINE_BUF_SIZE - 1) {
                ESP_LOGW(TAG, "RX overflow — discarding %u bytes", s_rx_pos);
                s_rx_pos = 0;
                continue;
            }
            s_rx_line[s_rx_pos++] = (char)b;
        }
    }
}

// ============================================================================
// OUTBOUND MESSAGE IMPLEMENTATIONS
// ============================================================================

void uart_master_send_hub_boot(void)
{
    char ts[10]; uptime_str(ts, sizeof(ts));

    hub_config_t *c = lock_config();
    if (!c) return;

    int  count      = c->sensor_count;
    const char *us  = unit_state_str(c->unit_state);
    unlock_config();

    tx_send_fmt("{\"type\":\"hub_boot\",\"sensor_count\":%d,"
                "\"unit_state\":\"%s\",\"ts_utc\":\"%s\"}",
                count, us, ts);
    ESP_LOGI(TAG, "TX hub_boot sent");
}

void uart_master_send_hub_ready(void)
{
    char ts[10]; uptime_str(ts, sizeof(ts));

    hub_config_t *c = lock_config();
    if (!c) return;

    int online  = 0;
    int offline = 0;
    for (int i = 0; i < c->sensor_count; i++) {
        if (c->sensors[i].online) online++;
        else                      offline++;
    }
    const char *us = unit_state_str(c->unit_state);
    unlock_config();

    tx_send_fmt("{\"type\":\"hub_ready\",\"online_count\":%d,"
                "\"offline_count\":%d,\"unit_state\":\"%s\","
                "\"ts_utc\":\"%s\"}",
                online, offline, us, ts);
    ESP_LOGI(TAG, "TX hub_ready online=%d offline=%d", online, offline);
}

void uart_master_send_unit_occupancy(const char *state)
{
    char ts[10]; uptime_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"unit_occupancy\",\"state\":\"%s\","
                "\"ts_utc\":\"%s\"}",
                state ? state : "VACANT", ts);
    ESP_LOGI(TAG, "TX unit_occupancy=%s", state ? state : "VACANT");
}

void uart_master_send_sensor_presence(const char *sensor_name,
                                      const char *model,
                                      bool        presence)
{
    char ts[10]; uptime_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"sensor_presence\",\"sensor\":\"%s\","
                "\"model\":\"%s\",\"state\":\"%s\",\"ts_utc\":\"%s\"}",
                sensor_name ? sensor_name : "",
                model       ? model       : "",
                presence ? "YES" : "NO", ts);
}

void uart_master_send_environment(const char *sensor_name,
                                  float        temp_c,
                                  float        humidity_pct)
{
    char ts[10]; uptime_str(ts, sizeof(ts));
    int32_t t = (int32_t)(temp_c       * 100.0f);
    int32_t h = (int32_t)(humidity_pct * 100.0f);
    tx_send_fmt("{\"type\":\"environment\",\"sensor\":\"%s\","
                "\"temp_c_x100\":%ld,\"hum_pct_x100\":%ld,"
                "\"ts_utc\":\"%s\"}",
                sensor_name ? sensor_name : "",
                (long)t, (long)h, ts);
    ESP_LOGI(TAG, "TX environment sensor=%s temp=%.2f hum=%.2f",
             sensor_name ? sensor_name : "",
             (double)temp_c, (double)humidity_pct);
}

void uart_master_send_door(const char *sensor_name, bool is_open)
{
    char ts[10]; uptime_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"door\",\"sensor\":\"%s\","
                "\"state\":\"%s\",\"ts_utc\":\"%s\"}",
                sensor_name ? sensor_name : "",
                is_open ? "OPEN" : "CLOSED", ts);
    ESP_LOGI(TAG, "TX door sensor=%s state=%s",
             sensor_name ? sensor_name : "",
             is_open ? "OPEN" : "CLOSED");
}

void uart_master_notify_door_state(int         sensor_idx,
                                   const char *sensor_name,
                                   bool        is_open)
{
    if (sensor_idx < 0 || sensor_idx >= MAX_SENSORS) return;
    door_alarm_t *da = &s_door_alarm[sensor_idx];

    if (is_open && !da->door_open) {
        da->door_open      = true;
        da->alarm_sent     = false;
        da->open_since_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    } else if (!is_open && da->door_open) {
        da->door_open = false;
        if (da->alarm_sent) {
            uart_master_send_door_alarm(sensor_name, "CLEAR", 0);
            da->alarm_sent = false;
        }
    }
}

void uart_master_send_door_alarm(const char *sensor_name,
                                 const char *alarm_state,
                                 uint32_t    duration_sec)
{
    char ts[10]; uptime_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"door_alarm\",\"sensor\":\"%s\","
                "\"state\":\"%s\",\"duration_sec\":%lu,"
                "\"ts_utc\":\"%s\"}",
                sensor_name ? sensor_name : "",
                alarm_state ? alarm_state : "ALARM",
                (unsigned long)duration_sec, ts);
    ESP_LOGW(TAG, "TX door_alarm sensor=%s state=%s duration=%lus",
             sensor_name ? sensor_name : "",
             alarm_state ? alarm_state : "ALARM",
             (unsigned long)duration_sec);
}

void uart_master_send_sensor_health(const char *sensor_name, bool is_online)
{
    char ts[10]; uptime_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"sensor_health\",\"sensor\":\"%s\","
                "\"state\":\"%s\",\"ts_utc\":\"%s\"}",
                sensor_name ? sensor_name : "",
                is_online ? "ONLINE" : "OFFLINE", ts);
    ESP_LOGI(TAG, "TX sensor_health sensor=%s state=%s",
             sensor_name ? sensor_name : "",
             is_online ? "ONLINE" : "OFFLINE");
}

void uart_master_send_battery(const char *sensor_name, uint8_t battery_pct)
{
    char ts[10]; uptime_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"battery\",\"sensor\":\"%s\","
                "\"battery_pct\":%u,\"ts_utc\":\"%s\"}",
                sensor_name ? sensor_name : "",
                (unsigned)battery_pct, ts);
    ESP_LOGI(TAG, "TX battery sensor=%s pct=%u%%",
             sensor_name ? sensor_name : "",
             (unsigned)battery_pct);
}

void uart_master_send_heartbeat(void)
{
    char buf[UART_MASTER_TX_MSG_SIZE];
    int  pos = 0;
    int  rem = (int)sizeof(buf) - 2;

#define HB(fmt, ...) \
    do { int _n = snprintf(buf + pos, (size_t)(rem - pos), \
                           fmt, ##__VA_ARGS__); \
         if (_n > 0 && (pos + _n) < rem) pos += _n; } while (0)

    char ts[10]; uptime_str(ts, sizeof(ts));

    hub_config_t *c = lock_config();
    if (!c) { ESP_LOGW(TAG, "heartbeat: lock failed"); return; }

    HB("{\"type\":\"heartbeat\",\"unit_state\":\"%s\","
       "\"ts_utc\":\"%s\",\"sensors\":[",
       unit_state_str(c->unit_state), ts);

    for (int i = 0; i < c->sensor_count; i++) {
        sensor_t     *s = &c->sensors[i];
        sensor_type_t t = (sensor_type_t)s->sensor_type;
        if (i > 0) HB(",");

        if (t == SENSOR_ZG_102Z || t == SENSOR_ZG_102ZA) {
            HB("{\"name\":\"%s\",\"online\":%s,"
               "\"contact\":\"%s\",\"battery\":%u}",
               s->sensor_name,
               s->online ? "true" : "false",
               s->contact_open ? "OPEN" : "CLOSED",
               s->battery_pct);
        } else {
            HB("{\"name\":\"%s\",\"online\":%s,"
               "\"presence\":%s,\"battery\":%u}",
               s->sensor_name,
               s->online ? "true" : "false",
               s->presence ? "true" : "false",
               s->battery_pct);
        }
    }

    /* Best temp/humidity from first online ZG-204ZV */
    float temp_c = 0.0f, hum_pct = 0.0f;
    for (int i = 0; i < c->sensor_count; i++) {
        if ((sensor_type_t)c->sensors[i].sensor_type == SENSOR_ZG_204ZV
            && c->sensors[i].online) {
            temp_c  = (float)c->sensors[i].temperature_cdeg / 100.0f;
            hum_pct = (float)c->sensors[i].humidity_cpct    / 100.0f;
            break;
        }
    }
    unlock_config();

    int32_t ti = (int32_t)(temp_c  * 100.0f);
    int32_t hi = (int32_t)(hum_pct * 100.0f);
    HB("],\"temp_c_x100\":%ld,\"hum_pct_x100\":%ld}",
       (long)ti, (long)hi);

#undef HB

    buf[pos]     = '\n';
    buf[pos + 1] = '\0';
    tx_enqueue(buf, (uint16_t)(pos + 1));
    ESP_LOGI(TAG, "TX heartbeat sent");
}

void uart_master_send_config_response(void)
{
    uart_hub_config_t cfg;
    uart_master_get_config(&cfg);
    char ts[10]; uptime_str(ts, sizeof(ts));

    char buf[UART_MASTER_TX_MSG_SIZE];
    int  pos = 0;
    int  rem = (int)sizeof(buf) - 2;

#define CR(fmt, ...) \
    do { int _n = snprintf(buf + pos, (size_t)(rem - pos), \
                           fmt, ##__VA_ARGS__); \
         if (_n > 0 && (pos + _n) < rem) pos += _n; } while (0)

    CR("{\"type\":\"config_response\","
       "\"pairing_duration_sec\":%u,"
       "\"watchdog_enable\":%s,"
       "\"watchdog_interval_min\":%u,"
       "\"watchdog_ping_timeout_sec\":%u,"
       "\"door_alarm_threshold_min\":%u,"
       "\"heartbeat_interval_min\":%u,"
       "\"ts_utc\":\"%s\","
       "\"sensors\":[",
       cfg.pairing_duration_sec,
       cfg.watchdog_enable ? "true" : "false",
       cfg.watchdog_interval_min,
       cfg.watchdog_ping_timeout_sec,
       cfg.door_alarm_threshold_min,
       cfg.heartbeat_interval_min,
       ts);

    hub_config_t *c = lock_config();
    if (c) {
        for (int i = 0; i < c->sensor_count; i++) {
            sensor_t     *s    = &c->sensors[i];
            sensor_role_t role = (sensor_role_t)s->sensor_role;
            if (i > 0) CR(",");
            CR("{\"index\":%d,\"name\":\"%s\","
               "\"model\":\"%s\",\"role\":\"%s\","
               "\"online\":%s,\"battery\":%u}",
               i, s->sensor_name,
               friendly_name_from_type((sensor_type_t)s->sensor_type),
               role == ROLE_DOOR ? "DOOR" : "PRESENCE",
               s->online ? "true" : "false",
               s->battery_pct);
        }
        unlock_config();
    }
    CR("]}");

#undef CR

    buf[pos]     = '\n';
    buf[pos + 1] = '\0';
    tx_enqueue(buf, (uint16_t)(pos + 1));
    ESP_LOGI(TAG, "TX config_response sent");
}

void uart_master_send_log_response(const char *log_line)
{
    if (!log_line) return;
    char ts[10]; uptime_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"log_response\",\"line\":\"%s\","
                "\"ts_utc\":\"%s\"}",
                log_line, ts);
}

void uart_master_send_ack(const char *command, bool success)
{
    if (!command) return;
    char ts[10]; uptime_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"ack\",\"command\":\"%s\","
                "\"status\":\"%s\",\"ts_utc\":\"%s\"}",
                command,
                success ? "ok" : "error",
                ts);
}

// ============================================================================
// TIMER TASK — door alarm + heartbeat, wakes every 1 second
// ============================================================================

static void door_alarm_tick(void)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    uart_hub_config_t cfg;
    uart_master_get_config(&cfg);
    uint32_t thresh = (uint32_t)cfg.door_alarm_threshold_min * 60u;

    hub_config_t *c = lock_config();
    if (!c) return;

    for (int i = 0; i < c->sensor_count; i++) {
        door_alarm_t *da = &s_door_alarm[i];
        if (!da->door_open || da->alarm_sent) continue;
        uint32_t elapsed = now - da->open_since_sec;
        if (elapsed >= thresh) {
            da->alarm_sent = true;
            char name[SENSOR_NAME_LEN];
            strncpy(name, c->sensors[i].sensor_name, SENSOR_NAME_LEN - 1);
            name[SENSOR_NAME_LEN - 1] = '\0';
            unlock_config();
            uart_master_send_door_alarm(name, "ALARM", elapsed);
            return;
        }
    }
    unlock_config();
}

static void heartbeat_tick(void)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    uart_hub_config_t cfg;
    uart_master_get_config(&cfg);
    uint32_t interval = (uint32_t)cfg.heartbeat_interval_min * 60u;
    if ((now - s_last_heartbeat_sec) >= interval) {
        s_last_heartbeat_sec = now;
        uart_master_send_heartbeat();
    }
}

static void uart_timer_task(void *arg)
{
    (void)arg;
    /* Stagger first heartbeat — avoid flood on boot */
    s_last_heartbeat_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        door_alarm_tick();
        heartbeat_tick();
    }
}

// ============================================================================
// INIT
// ============================================================================

esp_err_t uart_master_init(void)
{
    uart_master_load_config();

    s_cfg_mutex = xSemaphoreCreateMutexStatic(&s_cfg_mutex_cb);
    if (!s_cfg_mutex) {
        ESP_LOGE(TAG, "cfg mutex create failed");
        return ESP_FAIL;
    }

    s_tx_queue = xQueueCreateStatic(UART_MASTER_TX_QUEUE_DEPTH,
                                    sizeof(tx_msg_t),
                                    s_tx_queue_buf,
                                    &s_tx_queue_cb);
    if (!s_tx_queue) {
        ESP_LOGE(TAG, "TX queue create failed");
        return ESP_FAIL;
    }

    uart_config_t uc = {
        .baud_rate  = UART_MASTER_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;
    err = uart_param_config(UART_MASTER_PORT, &uc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_set_pin(UART_MASTER_PORT,
                       UART_MASTER_TX_PIN, UART_MASTER_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_driver_install(UART_MASTER_PORT,
                              UART_MASTER_RX_BUF_SIZE,
                              UART_MASTER_TX_BUF_SIZE,
                              0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err));
        return err;
    }

    memset(s_door_alarm, 0, sizeof(s_door_alarm));

    xTaskCreateStatic(uart_rx_task,    "uart_rx",
                      UART_MASTER_RX_TASK_STACK, NULL,
                      UART_MASTER_RX_TASK_PRIO,
                      s_rx_stack, &s_rx_tcb);

    xTaskCreateStatic(uart_tx_task,    "uart_tx",
                      UART_MASTER_TX_TASK_STACK, NULL,
                      UART_MASTER_TX_TASK_PRIO,
                      s_tx_stack, &s_tx_tcb);

    xTaskCreateStatic(uart_timer_task, "uart_tmr",
                      UART_MASTER_TMR_TASK_STACK, NULL,
                      UART_MASTER_TMR_TASK_PRIO,
                      s_tmr_stack, &s_tmr_tcb);

    ESP_LOGI(TAG, "UART Master ready — TX=GPIO%d RX=GPIO%d baud=%d",
             UART_MASTER_TX_PIN, UART_MASTER_RX_PIN, UART_MASTER_BAUD);
    return ESP_OK;
}