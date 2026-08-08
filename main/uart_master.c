/*
 * uart_master.c
 * Sensor Hub <-> Master UART Communication Layer
 * Innovatsii EMS — Pico 1  |  Firmware 0.2.5
 *
 * V4.2+:
 *   - UTC timestamps (g_utc_boot_epoch + uptime), set from hub_init
 *   - hub_aggregate replaces unit_occupancy
 *   - presence_fading_time_sec: 0 valid; motion_sensitivity global default
 *   - set_sensor_config command → per-sensor fading/sensitivity (Tuya DP)
 *   - config_response reports live fading_time/sensitivity per presence sensor
 *
 * CRITICAL: s_tx_dequeue_buf must remain a static global — never local.
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

static QueueHandle_t s_tx_queue = NULL;
static StaticQueue_t s_tx_queue_cb;
static uint8_t       s_tx_queue_buf[UART_MASTER_TX_QUEUE_DEPTH * sizeof(tx_msg_t)];

static StackType_t  s_tx_stack[UART_MASTER_TX_TASK_STACK];
static StaticTask_t s_tx_tcb;
static StackType_t  s_rx_stack[UART_MASTER_RX_TASK_STACK];
static StaticTask_t s_rx_tcb;
static StackType_t  s_tmr_stack[UART_MASTER_TMR_TASK_STACK];
static StaticTask_t s_tmr_tcb;

/* CRITICAL: must remain static global — never move to stack */
static tx_msg_t s_tx_dequeue_buf;

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

static uint32_t s_last_heartbeat_sec          = 0;
static uint32_t s_last_door_silence_check_sec = 0;

// ============================================================================
// UTC TIMESTAMP HELPER
// ============================================================================

extern volatile int64_t g_utc_boot_epoch;   /* main.c */

static uint64_t uptime_sec(void)
{
    return esp_timer_get_time() / 1000000ULL;
}

static void fmt_utc_epoch(char *buf, size_t len, int64_t epoch)
{
    time_t t = (time_t)epoch;
    struct tm tm_info;
    gmtime_r(&t, &tm_info);
    int year = tm_info.tm_year + 1900;
    if (year < 2000) year = 2000;
    if (year > 2099) year = 2099;
    snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d",
             year, tm_info.tm_mon + 1, tm_info.tm_mday,
             tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);
}

static void utc_str(char *buf, size_t len)
{
    if (g_utc_boot_epoch > 0) {
        int64_t now_epoch = g_utc_boot_epoch + (int64_t)uptime_sec();
        fmt_utc_epoch(buf, len, now_epoch);
    } else {
        uint64_t sec = uptime_sec();
        snprintf(buf, len, "%02u:%02u:%02u",
                 (unsigned)(sec / 3600), (unsigned)((sec % 3600) / 60),
                 (unsigned)(sec % 60));
    }
}

// ============================================================================
// DEFAULT CONFIG AND CLAMPING
// ============================================================================

static const uart_hub_config_t k_defaults = {
    .pairing_duration_sec           = 120,
    .watchdog_enable                = true,
    .watchdog_interval_min          = 60,
    .watchdog_ping_timeout_sec      = 30,
    .door_alarm_threshold_min       = 10,
    .heartbeat_interval_min         = 30,
    .presence_fading_time_sec       = 30,
    .door_sensor_max_silence_hours  = 24,
    .motion_sensitivity             = 9,
};

static void config_clamp(uart_hub_config_t *c)
{
    if (c->pairing_duration_sec          < 30   ) c->pairing_duration_sec          = 30;
    if (c->pairing_duration_sec          > 600  ) c->pairing_duration_sec          = 600;
    if (c->watchdog_interval_min         < 1    ) c->watchdog_interval_min         = 1;
    if (c->watchdog_interval_min         > 1440 ) c->watchdog_interval_min         = 1440;
    if (c->watchdog_ping_timeout_sec     < 10   ) c->watchdog_ping_timeout_sec     = 10;
    if (c->watchdog_ping_timeout_sec     > 120  ) c->watchdog_ping_timeout_sec     = 120;
    if (c->door_alarm_threshold_min      < 1    ) c->door_alarm_threshold_min      = 1;
    if (c->door_alarm_threshold_min      > 60   ) c->door_alarm_threshold_min      = 60;
    if (c->heartbeat_interval_min        < 1    ) c->heartbeat_interval_min        = 1;
    if (c->heartbeat_interval_min        > 1440 ) c->heartbeat_interval_min        = 1440;
    if (c->presence_fading_time_sec      > 28800) c->presence_fading_time_sec      = 28800;
    if (c->door_sensor_max_silence_hours < 1    ) c->door_sensor_max_silence_hours = 1;
    if (c->door_sensor_max_silence_hours > 72   ) c->door_sensor_max_silence_hours = 72;
    if (c->motion_sensitivity            > 19   ) c->motion_sensitivity            = 19;
}

// ============================================================================
// NVS CONFIG
// ============================================================================

#define NVS_NS  "uart_hub"
#define NVS_KEY "cfg_v4"

esp_err_t uart_master_load_config(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No UART config in NVS — using defaults");
        s_config = k_defaults;
        return ESP_OK;
    }
    if (err != ESP_OK) { s_config = k_defaults; return err; }
    size_t sz = sizeof(uart_hub_config_t);
    err = nvs_get_blob(h, NVS_KEY, &s_config, &sz);
    nvs_close(h);
    if (err != ESP_OK || sz != sizeof(uart_hub_config_t)) {
        ESP_LOGW(TAG, "UART config blob invalid — using defaults");
        s_config = k_defaults;
        return ESP_OK;
    }
    config_clamp(&s_config);
    return ESP_OK;
}

esp_err_t uart_master_save_config(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(h, NVS_KEY, &s_config, sizeof(uart_hub_config_t));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

void uart_master_get_config(uart_hub_config_t *out)
{
    if (!out) return;
    if (xSemaphoreTake(s_cfg_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        *out = s_config;
        xSemaphoreGive(s_cfg_mutex);
    } else {
        *out = k_defaults;
    }
}

esp_err_t uart_master_set_config(const uart_hub_config_t *in)
{
    if (!in) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_cfg_mutex, pdMS_TO_TICKS(500)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    s_config = *in;
    config_clamp(&s_config);
    xSemaphoreGive(s_cfg_mutex);
    return uart_master_save_config();
}

// ============================================================================
// TX QUEUE AND HELPERS
// ============================================================================

static void tx_enqueue(const char *data, uint16_t len)
{
    if (!s_tx_queue || len == 0 || len >= UART_MASTER_TX_MSG_SIZE) return;
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
    if (n <= 0 || n >= (int)(sizeof(buf) - 2)) return;
    buf[n]     = '\n';
    buf[n + 1] = '\0';
    tx_enqueue(buf, (uint16_t)(n + 1));
}

static void uart_tx_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (xQueueReceive(s_tx_queue, &s_tx_dequeue_buf, portMAX_DELAY) == pdTRUE) {
            if (s_tx_dequeue_buf.len > 0 &&
                s_tx_dequeue_buf.len < UART_MASTER_TX_MSG_SIZE)
                uart_write_bytes(UART_MASTER_PORT,
                                 s_tx_dequeue_buf.data, s_tx_dequeue_buf.len);
        }
    }
}

// ============================================================================
// JSON FIELD EXTRACTORS
// ============================================================================

static bool json_str(const char *json, const char *key, char *out, size_t out_sz)
{
    char pat[66];
    int n = snprintf(pat, sizeof(pat), "\"%s\":", key);
    if (n <= 0 || n >= (int)sizeof(pat)) return false;
    const char *p = json;
    while ((p = strstr(p, pat)) != NULL) {
        if (p > json) {
            char prev = *(p - 1);
            if (prev != '{' && prev != ',' && prev != ' ' && prev != '\n') { p++; continue; }
        }
        p += strlen(pat);
        while (*p == ' ' || *p == '\t') p++;
        if (*p != '"') return false;
        p++;
        const char *e = strchr(p, '"');
        if (!e) return false;
        size_t len = (size_t)(e - p);
        if (len >= out_sz) return false;
        memcpy(out, p, len);
        out[len] = '\0';
        return true;
    }
    return false;
}

static bool json_int(const char *json, const char *key, int32_t *out)
{
    char pat[66];
    int n = snprintf(pat, sizeof(pat), "\"%s\":", key);
    if (n <= 0 || n >= (int)sizeof(pat)) return false;
    const char *p = json;
    while ((p = strstr(p, pat)) != NULL) {
        if (p > json) {
            char prev = *(p - 1);
            if (prev != '{' && prev != ',' && prev != ' ' && prev != '\n') { p++; continue; }
        }
        p += strlen(pat);
        while (*p == ' ' || *p == '\t') p++;
        char *ep;
        long v = strtol(p, &ep, 10);
        if (ep == p) return false;
        *out = (int32_t)v;
        return true;
    }
    return false;
}

static bool json_int64(const char *json, const char *key, int64_t *out)
{
    char pat[66];
    int n = snprintf(pat, sizeof(pat), "\"%s\":", key);
    if (n <= 0 || n >= (int)sizeof(pat)) return false;
    const char *p = json;
    while ((p = strstr(p, pat)) != NULL) {
        if (p > json) {
            char prev = *(p - 1);
            if (prev != '{' && prev != ',' && prev != ' ' && prev != '\n') { p++; continue; }
        }
        p += strlen(pat);
        while (*p == ' ' || *p == '\t') p++;
        char *ep;
        long long v = strtoll(p, &ep, 10);
        if (ep == p) return false;
        *out = (int64_t)v;
        return true;
    }
    return false;
}

static bool json_bool(const char *json, const char *key, bool *out)
{
    char pat[66];
    int n = snprintf(pat, sizeof(pat), "\"%s\":", key);
    if (n <= 0 || n >= (int)sizeof(pat)) return false;
    const char *p = json;
    while ((p = strstr(p, pat)) != NULL) {
        if (p > json) {
            char prev = *(p - 1);
            if (prev != '{' && prev != ',' && prev != ' ' && prev != '\n') { p++; continue; }
        }
        p += strlen(pat);
        while (*p == ' ' || *p == '\t') p++;
        if (strncmp(p, "true",  4) == 0) { *out = true;  return true; }
        if (strncmp(p, "false", 5) == 0) { *out = false; return true; }
        return false;
    }
    return false;
}

// ============================================================================
// COMMAND HANDLERS
// ============================================================================

static void cmd_ping             (const char *json, uint16_t len);
static void cmd_hub_init         (const char *json, uint16_t len);
static void cmd_start_watchdog   (const char *json, uint16_t len);
static void cmd_set_sensor_name  (const char *json, uint16_t len);
static void cmd_set_sensor_config(const char *json, uint16_t len);
static void cmd_set_config       (const char *json, uint16_t len);
static void cmd_get_config       (const char *json, uint16_t len);
static void cmd_get_logs         (const char *json, uint16_t len);
static void cmd_start_pairing    (const char *json, uint16_t len);
static void cmd_stop_pairing     (const char *json, uint16_t len);
static void cmd_remove_sensor    (const char *json, uint16_t len);
static void cmd_factory_reset    (const char *json, uint16_t len);
static void cmd_restart          (const char *json, uint16_t len);

static const cmd_entry_t k_cmds[] = {
    { "ping",              cmd_ping              },
    { "hub_init",          cmd_hub_init          },
    { "start_watchdog",    cmd_start_watchdog    },
    { "set_sensor_name",   cmd_set_sensor_name   },
    { "set_sensor_config", cmd_set_sensor_config },
    { "set_config",        cmd_set_config        },
    { "get_config",        cmd_get_config        },
    { "get_logs",          cmd_get_logs          },
    { "start_pairing",     cmd_start_pairing     },
    { "stop_pairing",      cmd_stop_pairing      },
    { "remove_sensor",     cmd_remove_sensor     },
    { "factory_reset",     cmd_factory_reset     },
    { "restart",           cmd_restart           },
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

static void cmd_ping(const char *json, uint16_t len)
{
    (void)json; (void)len;
    uart_master_send_pong();
    ESP_LOGI(TAG, "ping received — pong sent");
}

volatile bool g_hub_init_received   = false;
volatile bool g_hub_init_mode_debug = false;

static void cmd_hub_init(const char *json, uint16_t len)
{
    (void)len;
    uart_master_send_ack("hub_init", true);

    int64_t epoch = 0;
    if (json_int64(json, "utc_epoch", &epoch) && epoch > 0) {
        g_utc_boot_epoch = epoch - (int64_t)uptime_sec();
        char ts[32]; utc_str(ts, sizeof(ts));
        ESP_LOGI(TAG, "UTC epoch received: %lld → current UTC: %s",
                 (long long)epoch, ts);
    }

    uart_hub_config_t nc;
    uart_master_get_config(&nc);
    int32_t tmp; bool btmp;
    if (json_int(json,  "pairing_duration_sec",          &tmp))  nc.pairing_duration_sec          = (uint16_t)tmp;
    if (json_bool(json, "watchdog_enable",               &btmp)) nc.watchdog_enable               = btmp;
    if (json_int(json,  "watchdog_interval_min",         &tmp))  nc.watchdog_interval_min         = (uint16_t)tmp;
    if (json_int(json,  "watchdog_ping_timeout_sec",     &tmp))  nc.watchdog_ping_timeout_sec     = (uint16_t)tmp;
    if (json_int(json,  "door_alarm_threshold_min",      &tmp))  nc.door_alarm_threshold_min      = (uint16_t)tmp;
    if (json_int(json,  "heartbeat_interval_min",        &tmp))  nc.heartbeat_interval_min        = (uint16_t)tmp;
    if (json_int(json,  "presence_fading_time_sec",      &tmp))  nc.presence_fading_time_sec      = (uint16_t)tmp;
    if (json_int(json,  "door_sensor_max_silence_hours", &tmp))  nc.door_sensor_max_silence_hours = (uint16_t)tmp;
    if (json_int(json,  "motion_sensitivity",            &tmp))  nc.motion_sensitivity            = (uint16_t)tmp;
    uart_master_set_config(&nc);

    char mode_str[16] = {0};
    json_str(json, "mode", mode_str, sizeof(mode_str));
    g_hub_init_mode_debug = (strcmp(mode_str, "debug") == 0);

    g_hub_init_received = true;
    ESP_LOGI(TAG, "hub_init received — config stored, Zigbee start signalled");
}

static void cmd_start_watchdog(const char *json, uint16_t len)
{
    (void)json; (void)len;
    g_watchdog_started = true;
    uart_master_send_ack("start_watchdog", true);
    ESP_LOGI(TAG, "start_watchdog received — watchdog and data flow active");
}

static void cmd_set_sensor_name(const char *json, uint16_t len)
{
    (void)len;
    int32_t idx = -1;
    char    name[SENSOR_NAME_LEN];
    if (!json_int(json, "sensor_index", &idx) || idx < 0 || idx >= MAX_SENSORS) {
        uart_master_send_ack("set_sensor_name", false); return;
    }
    if (!json_str(json, "name", name, sizeof(name))) {
        uart_master_send_ack("set_sensor_name", false); return;
    }
    hub_config_t *c = lock_config();
    if (!c) { uart_master_send_ack("set_sensor_name", false); return; }
    if ((int)idx >= c->sensor_count) {
        unlock_config(); uart_master_send_ack("set_sensor_name", false); return;
    }
    strncpy(c->sensors[idx].sensor_name, name, SENSOR_NAME_LEN - 1);
    c->sensors[idx].sensor_name[SENSOR_NAME_LEN - 1] = '\0';
    unlock_config();
    mark_dirty();
    ESP_LOGI(TAG, "Sensor %ld renamed to '%s'", (long)idx, name);
    uart_master_send_ack("set_sensor_name", true);
}

/*
 * set_sensor_config — push fading_time / motion sensitivity to a presence
 * sensor. Omit or set -1 for a field to leave it unchanged.
 * JSON: {"type":"set_sensor_config","sensor_index":N,
 *        "fading_time":F,"sensitivity":S}
 */
static void cmd_set_sensor_config(const char *json, uint16_t len)
{
    (void)len;
    int32_t idx = -1;
    if (!json_int(json, "sensor_index", &idx) || idx < 0 || idx >= MAX_SENSORS) {
        uart_master_send_ack("set_sensor_config", false); return;
    }
    int32_t fading = -1, sens = -1;
    (void)json_int(json, "fading_time", &fading);
    (void)json_int(json, "sensitivity", &sens);

    hub_config_t *c = lock_config();
    if (!c || (int)idx >= c->sensor_count) {
        if (c) unlock_config();
        uart_master_send_ack("set_sensor_config", false);
        return;
    }
    sensor_type_t t = (sensor_type_t)c->sensors[idx].sensor_type;
    unlock_config();

    if (t != SENSOR_ZG_204ZL) {
        ESP_LOGW(TAG, "set_sensor_config: idx %ld not a PIR sensor", (long)idx);
        uart_master_send_ack("set_sensor_config", false);
        return;
    }

    extern void uart_cmd_set_sensor_config(int idx, int fading_sec, int sensitivity);
    uart_cmd_set_sensor_config((int)idx, (int)fading, (int)sens);

    uart_master_send_ack("set_sensor_config", true);
    ESP_LOGI(TAG, "set_sensor_config idx=%ld fading=%ld sensitivity=%ld",
             (long)idx, (long)fading, (long)sens);
}

static void cmd_set_config(const char *json, uint16_t len)
{
    (void)len;
    uart_hub_config_t nc;
    uart_master_get_config(&nc);
    int32_t tmp; bool btmp;
    if (json_int(json,  "pairing_duration_sec",          &tmp))  nc.pairing_duration_sec          = (uint16_t)tmp;
    if (json_bool(json, "watchdog_enable",               &btmp)) nc.watchdog_enable               = btmp;
    if (json_int(json,  "watchdog_interval_min",         &tmp))  nc.watchdog_interval_min         = (uint16_t)tmp;
    if (json_int(json,  "watchdog_ping_timeout_sec",     &tmp))  nc.watchdog_ping_timeout_sec     = (uint16_t)tmp;
    if (json_int(json,  "door_alarm_threshold_min",      &tmp))  nc.door_alarm_threshold_min      = (uint16_t)tmp;
    if (json_int(json,  "heartbeat_interval_min",        &tmp))  nc.heartbeat_interval_min        = (uint16_t)tmp;
    if (json_int(json,  "presence_fading_time_sec",      &tmp))  nc.presence_fading_time_sec      = (uint16_t)tmp;
    if (json_int(json,  "door_sensor_max_silence_hours", &tmp))  nc.door_sensor_max_silence_hours = (uint16_t)tmp;
    if (json_int(json,  "motion_sensitivity",            &tmp))  nc.motion_sensitivity            = (uint16_t)tmp;
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
    uart_master_send_log_response("Log ring buffer — Phase 9");
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
    g_new_sensor_count = 0;
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
    if (!json_int(json, "sensor_index", &idx) || idx < 0 || idx >= MAX_SENSORS) {
        uart_master_send_ack("remove_sensor", false); return;
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
// ============================================================================

static void uart_rx_task(void *arg)
{
    (void)arg;
    uint8_t chunk[128];
    for (;;) {
        int bytes = uart_read_bytes(UART_MASTER_PORT, chunk, sizeof(chunk),
                                    pdMS_TO_TICKS(20));
        for (int i = 0; i < bytes; i++) {
            uint8_t b = chunk[i];
            if (b == '\r') continue;
            if (b == '\n') {
                if (s_rx_pos > 0) {
                    s_rx_line[s_rx_pos] = '\0';
                    if (s_rx_line[0] == '{') {
                        ESP_LOGI(TAG, "RX: %s", s_rx_line);
                        dispatch_command(s_rx_line, s_rx_pos);
                    } else {
                        ESP_LOGW(TAG, "RX: non-JSON discarded (starts 0x%02x)",
                                 (uint8_t)s_rx_line[0]);
                    }
                    s_rx_pos = 0;
                }
                continue;
            }
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
// OUTBOUND — Boot Protocol
// ============================================================================

void uart_master_send_pong(void)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"pong\",\"firmware_version\":\"%s\",\"ts_utc\":\"%s\"}",
                FIRMWARE_VERSION, ts);
}

void uart_master_send_hub_ready(void)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    hub_config_t *c = lock_config();
    if (!c) return;
    int online = 0, offline = 0;
    for (int i = 0; i < c->sensor_count; i++)
        if (c->sensors[i].online) online++; else offline++;
    bool needs_pairing = (c->sensor_count == 0);
    const char *agg = hub_aggregate_str(c->hub_status.aggregate);
    unlock_config();
    tx_send_fmt("{\"type\":\"hub_ready\",\"firmware_version\":\"%s\","
                "\"sensor_count\":%d,\"online_count\":%d,\"offline_count\":%d,"
                "\"hub_aggregate\":\"%s\",\"needs_pairing\":%s,\"ts_utc\":\"%s\"}",
                FIRMWARE_VERSION, online + offline, online, offline, agg,
                needs_pairing ? "true" : "false", ts);
    ESP_LOGI(TAG, "TX hub_ready online=%d offline=%d needs_pairing=%s",
             online, offline, needs_pairing ? "true" : "false");
}

void uart_master_send_sensor_joined(int idx, const char *name, const char *model,
                                    const char *role, bool online, uint8_t batt)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"sensor_joined\",\"index\":%d,\"name\":\"%s\","
                "\"model\":\"%s\",\"role\":\"%s\",\"online\":%s,\"battery\":%u,"
                "\"ts_utc\":\"%s\"}",
                idx, name ? name : "", model ? model : "", role ? role : "",
                online ? "true" : "false", (unsigned)batt, ts);
}

void uart_master_send_sensor_status(int idx, const char *name, const char *model,
                                    const char *role, bool online)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"sensor_status\",\"index\":%d,\"name\":\"%s\","
                "\"model\":\"%s\",\"role\":\"%s\",\"online\":%s,\"ts_utc\":\"%s\"}",
                idx, name ? name : "", model ? model : "", role ? role : "",
                online ? "true" : "false", ts);
}

void uart_master_send_sensor_list_complete(int total, int online, int offline)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"sensor_list_complete\",\"total\":%d,\"online\":%d,"
                "\"offline\":%d,\"ts_utc\":\"%s\"}", total, online, offline, ts);
    ESP_LOGI(TAG, "TX sensor_list_complete total=%d online=%d offline=%d",
             total, online, offline);
}

void uart_master_send_new_sensor_joined(int idx, const char *name,
                                        const char *model, const char *role)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"new_sensor_joined\",\"index\":%d,\"name\":\"%s\","
                "\"model\":\"%s\",\"role\":\"%s\",\"ts_utc\":\"%s\"}",
                idx, name ? name : "", model ? model : "", role ? role : "", ts);
    ESP_LOGI(TAG, "TX new_sensor_joined idx=%d name=%s model=%s",
             idx, name ? name : "?", model ? model : "?");
}

void uart_master_send_pairing_complete(int new_sensors, int total_sensors)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"pairing_complete\",\"new_sensors\":%d,"
                "\"total_sensors\":%d,\"ts_utc\":\"%s\"}",
                new_sensors, total_sensors, ts);
}

// ============================================================================
// OUTBOUND — Runtime
// ============================================================================

void uart_master_send_hub_aggregate(const char *agg_state)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"hub_aggregate\",\"state\":\"%s\",\"ts_utc\":\"%s\"}",
                agg_state ? agg_state : "VACANT", ts);
    ESP_LOGI(TAG, "TX hub_aggregate=%s", agg_state ? agg_state : "VACANT");
}

void uart_master_send_sensor_presence(const char *name, const char *model, bool p)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"sensor_presence\",\"sensor\":\"%s\",\"model\":\"%s\","
                "\"state\":\"%s\",\"ts_utc\":\"%s\"}",
                name ? name : "", model ? model : "", p ? "YES" : "NO", ts);
}

void uart_master_send_door(const char *name, bool is_open)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"door\",\"sensor\":\"%s\",\"state\":\"%s\","
                "\"ts_utc\":\"%s\"}",
                name ? name : "", is_open ? "OPEN" : "CLOSED", ts);
    ESP_LOGI(TAG, "TX door sensor=%s state=%s",
             name ? name : "", is_open ? "OPEN" : "CLOSED");
}

void uart_master_notify_door_state(int idx, const char *name, bool is_open)
{
    if (idx < 0 || idx >= MAX_SENSORS) return;
    door_alarm_t *da = &s_door_alarm[idx];
    if (is_open && !da->door_open) {
        da->door_open = true; da->alarm_sent = false;
        da->open_since_sec = (uint32_t)uptime_sec();
    } else if (!is_open && da->door_open) {
        da->door_open = false;
        if (da->alarm_sent) {
            uart_master_send_door_alarm(name, "CLEAR", 0);
            da->alarm_sent = false;
        }
    }
}

void uart_master_send_door_alarm(const char *name, const char *state, uint32_t dur)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"door_alarm\",\"sensor\":\"%s\",\"state\":\"%s\","
                "\"duration_sec\":%lu,\"ts_utc\":\"%s\"}",
                name ? name : "", state ? state : "ALARM",
                (unsigned long)dur, ts);
}

void uart_master_send_sensor_health(const char *name, bool online)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"sensor_health\",\"sensor\":\"%s\",\"state\":\"%s\","
                "\"ts_utc\":\"%s\"}",
                name ? name : "", online ? "ONLINE" : "OFFLINE", ts);
    ESP_LOGI(TAG, "TX sensor_health sensor=%s state=%s",
             name ? name : "", online ? "ONLINE" : "OFFLINE");
}

void uart_master_send_battery(const char *name, uint8_t batt)
{
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"battery\",\"sensor\":\"%s\",\"battery_pct\":%u,"
                "\"ts_utc\":\"%s\"}", name ? name : "", (unsigned)batt, ts);
    ESP_LOGI(TAG, "TX battery sensor=%s pct=%u%%", name ? name : "", (unsigned)batt);
}

void uart_master_send_heartbeat(void)
{
    if (!g_watchdog_started) return;
    char buf[UART_MASTER_TX_MSG_SIZE];
    int pos = 0, rem = (int)sizeof(buf) - 2;
#define HB(fmt, ...) do { int _n = snprintf(buf + pos, (size_t)(rem - pos), \
        fmt, ##__VA_ARGS__); if (_n > 0 && (pos + _n) < rem) pos += _n; } while (0)

    char ts[32]; utc_str(ts, sizeof(ts));
    hub_config_t *c = lock_config();
    if (!c) return;
    const char *agg = hub_aggregate_str(c->hub_status.aggregate);

    HB("{\"type\":\"heartbeat\",\"firmware_version\":\"%s\","
       "\"hub_aggregate\":\"%s\",\"ts_utc\":\"%s\",\"sensors\":[",
       FIRMWARE_VERSION, agg, ts);

    for (int i = 0; i < c->sensor_count; i++) {
        sensor_t *s = &c->sensors[i];
        sensor_type_t t = (sensor_type_t)s->sensor_type;
        if (i > 0) HB(",");
        char s_ts[32] = "unknown";
        if (s->last_seen > 0 && g_utc_boot_epoch > 0) {
            int64_t e = g_utc_boot_epoch + (int64_t)s->last_seen;
            fmt_utc_epoch(s_ts, sizeof(s_ts), e);
        }
        if (t == SENSOR_ZG_102Z || t == SENSOR_ZG_102ZA) {
            HB("{\"name\":\"%s\",\"model\":\"%s\",\"role\":\"DOOR\","
               "\"online\":%s,\"contact\":\"%s\",\"battery\":%u,"
               "\"last_seen_utc\":\"%s\"}",
               s->sensor_name, friendly_name_from_type(t),
               s->online ? "true" : "false",
               s->contact_open ? "OPEN" : "CLOSED",
               s->battery_pct, s_ts);
        } else {
            /* PRESENCE sensor (ZG-204ZL PIR).
             * keep_time_sec and sensitivity reported; no temp/hum. */
            HB("{\"name\":\"%s\",\"model\":\"%s\",\"role\":\"PRESENCE\","
               "\"online\":%s,\"presence\":%s,\"battery\":%u,"
               "\"keep_time_sec\":%u,\"sensitivity\":%u,"
               "\"last_seen_utc\":\"%s\"}",
               s->sensor_name, friendly_name_from_type(t),
               s->online ? "true" : "false",
               s->presence ? "true" : "false",
               s->battery_pct,
               (unsigned)g_meta[i].keep_time_sec, (unsigned)g_meta[i].sensitivity,
               s_ts);
        }
    }
    unlock_config();
    HB("]}");
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
    char ts[32]; utc_str(ts, sizeof(ts));
    char buf[UART_MASTER_TX_MSG_SIZE];
    int pos = 0, rem = (int)sizeof(buf) - 2;
#define CR(fmt, ...) do { int _n = snprintf(buf + pos, (size_t)(rem - pos), \
        fmt, ##__VA_ARGS__); if (_n > 0 && (pos + _n) < rem) pos += _n; } while (0)

    CR("{\"type\":\"config_response\",\"firmware_version\":\"%s\","
       "\"pairing_duration_sec\":%u,\"watchdog_enable\":%s,"
       "\"watchdog_interval_min\":%u,\"watchdog_ping_timeout_sec\":%u,"
       "\"door_alarm_threshold_min\":%u,\"heartbeat_interval_min\":%u,"
       "\"presence_fading_time_sec\":%u,\"door_sensor_max_silence_hours\":%u,"
       "\"motion_sensitivity\":%u,\"ts_utc\":\"%s\",\"sensors\":[",
       FIRMWARE_VERSION, cfg.pairing_duration_sec,
       cfg.watchdog_enable ? "true" : "false",
       cfg.watchdog_interval_min, cfg.watchdog_ping_timeout_sec,
       cfg.door_alarm_threshold_min, cfg.heartbeat_interval_min,
       cfg.presence_fading_time_sec, cfg.door_sensor_max_silence_hours,
       cfg.motion_sensitivity, ts);

    hub_config_t *c = lock_config();
    if (c) {
        for (int i = 0; i < c->sensor_count; i++) {
            sensor_t *s = &c->sensors[i];
            sensor_type_t st = (sensor_type_t)s->sensor_type;
            sensor_role_t role = (sensor_role_t)s->sensor_role;
            bool is_pir = (st == SENSOR_ZG_204ZL);
            if (i > 0) CR(",");
            CR("{\"index\":%d,\"name\":\"%s\",\"model\":\"%s\",\"role\":\"%s\","
               "\"online\":%s,\"battery\":%u,\"keep_time_sec\":%u,"
               "\"sensitivity\":%u,\"supports_config\":%s}",
               i, s->sensor_name, friendly_name_from_type(st),
               role == ROLE_DOOR ? "DOOR" : "PRESENCE",
               s->online ? "true" : "false", s->battery_pct,
               (unsigned)g_meta[i].keep_time_sec, (unsigned)g_meta[i].sensitivity,
               is_pir ? "true" : "false");
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
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"log_response\",\"line\":\"%s\",\"ts_utc\":\"%s\"}",
                log_line, ts);
}

void uart_master_send_ack(const char *command, bool success)
{
    if (!command) return;
    char ts[32]; utc_str(ts, sizeof(ts));
    tx_send_fmt("{\"type\":\"ack\",\"command\":\"%s\",\"status\":\"%s\","
                "\"ts_utc\":\"%s\"}",
                command, success ? "ok" : "error", ts);
}

// ============================================================================
// TIMER TASK — door alarm, heartbeat, door silence alert
// ============================================================================

static void door_alarm_tick(void)
{
    uint32_t now = (uint32_t)uptime_sec();
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

static void door_silence_tick(void)
{
    uint32_t now = (uint32_t)uptime_sec();
    if ((now - s_last_door_silence_check_sec) < 3600u) return;
    s_last_door_silence_check_sec = now;
    uart_hub_config_t cfg;
    uart_master_get_config(&cfg);
    time_t thr = (time_t)cfg.door_sensor_max_silence_hours * 3600;
    hub_config_t *c = lock_config();
    if (!c) return;
    for (int i = 0; i < c->sensor_count; i++) {
        if ((sensor_role_t)c->sensors[i].sensor_role != ROLE_DOOR) continue;
        time_t last = c->sensors[i].last_seen;
        if (last == 0) continue;
        if ((time(NULL) - last) > thr) {
            char name[SENSOR_NAME_LEN];
            strncpy(name, c->sensors[i].sensor_name, SENSOR_NAME_LEN - 1);
            name[SENSOR_NAME_LEN - 1] = '\0';
            unlock_config();
            uart_master_send_sensor_health(name, false);
            return;
        }
    }
    unlock_config();
}

static void heartbeat_tick(void)
{
    if (!g_watchdog_started) return;
    uint32_t now = (uint32_t)uptime_sec();
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
    s_last_heartbeat_sec          = (uint32_t)uptime_sec();
    s_last_door_silence_check_sec = s_last_heartbeat_sec;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        door_alarm_tick();
        door_silence_tick();
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
    if (!s_cfg_mutex) { ESP_LOGE(TAG, "cfg mutex create failed"); return ESP_FAIL; }

    s_tx_queue = xQueueCreateStatic(UART_MASTER_TX_QUEUE_DEPTH, sizeof(tx_msg_t),
                                    s_tx_queue_buf, &s_tx_queue_cb);
    if (!s_tx_queue) { ESP_LOGE(TAG, "TX queue create failed"); return ESP_FAIL; }

    uart_config_t uc = {
        .baud_rate  = UART_MASTER_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err;
    if ((err = uart_param_config(UART_MASTER_PORT, &uc)) != ESP_OK) return err;
    if ((err = uart_set_pin(UART_MASTER_PORT, UART_MASTER_TX_PIN, UART_MASTER_RX_PIN,
                            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) != ESP_OK) return err;
    if ((err = uart_driver_install(UART_MASTER_PORT, UART_MASTER_RX_BUF_SIZE,
                                   UART_MASTER_TX_BUF_SIZE, 0, NULL, 0)) != ESP_OK) return err;

    memset(s_door_alarm, 0, sizeof(s_door_alarm));
    memset(&s_tx_dequeue_buf, 0, sizeof(s_tx_dequeue_buf));

    xTaskCreateStatic(uart_rx_task,   "uart_rx",  UART_MASTER_RX_TASK_STACK, NULL,
                      UART_MASTER_RX_TASK_PRIO, s_rx_stack, &s_rx_tcb);
    xTaskCreateStatic(uart_tx_task,   "uart_tx",  UART_MASTER_TX_TASK_STACK, NULL,
                      UART_MASTER_TX_TASK_PRIO, s_tx_stack, &s_tx_tcb);
    xTaskCreateStatic(uart_timer_task,"uart_tmr", UART_MASTER_TMR_TASK_STACK, NULL,
                      UART_MASTER_TMR_TASK_PRIO, s_tmr_stack, &s_tmr_tcb);

    ESP_LOGI(TAG, "UART Master ready — TX=GPIO%d RX=GPIO%d baud=%d "
             "fading=%us sensitivity=%u door_silence=%uh",
             UART_MASTER_TX_PIN, UART_MASTER_RX_PIN, UART_MASTER_BAUD,
             s_config.presence_fading_time_sec, s_config.motion_sensitivity,
             s_config.door_sensor_max_silence_hours);
    return ESP_OK;
}