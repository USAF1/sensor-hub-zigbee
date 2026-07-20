/*
 * Sensor Hub Zigbee Coordinator
 *
 * Strict per-model build
 * Confirmed sensor modelID strings (from Zigbee2MQTT):
 *   ZG-204ZV              — HOBEIAN mmWave: IAS presence + Temp + Hum + Lux + Batt
 *   CK-BL702-MWS-01(7016) — HOBEIAN mmWave: IAS presence + Occupancy + Lux
 *   ZG-102Z               — HOBEIAN door:   IAS contact + Tamper + Batt
 *   ZG-102ZA              — HOBEIAN door:   IAS contact + Tamper + Batt
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_zigbee.h"
#include "ezbee/core.h"
#include "ezbee/nwk.h"
#include "ezbee/aps.h"
#include "ezbee/af.h"
#include "ezbee/zdo.h"
#include "ezbee/bdb.h"
#include "ezbee/secur.h"
#include "ezbee/app_signals.h"
#include "ezbee/zcl/zcl_core.h"
#include "ezbee/zcl/zcl_desc.h"
#include "ezbee/zcl/zcl_type.h"
#include "ezbee/zcl/zcl_common.h"
#include "ezbee/zcl/zcl_general_cmd.h"
#include "ezbee/zcl/cluster/ias_zone.h"
#include "ezbee/zdo/zdo_bind_mgmt.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"
#include "ezbee/zdo/zdo_nwk_mgmt.h"

#include "main.h"
#include "zigbee_gateway.h"

// ============================================================================
// COMPILE-TIME FLAGS
// ============================================================================

#ifndef ESP_ZIGBEE_STORAGE_PARTITION_NAME
#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "zb_storage"
#endif

#define TAG "SENSOR_HUB"

#define FACTORY_RESET_MODE  0   /* 1 = erase NVS on boot                        */
#define RAW_LOGS_MODE       1   /* 1 = developer (all logs), 0 = production only */
#define WATCHDOG_ENABLE     0   /* 0 = disabled (debug),     1 = production      */

#define PAIRING_DURATION_SEC          120
#define ZIGBEE_PRIMARY_CHANNEL_MASK   0x07FFF800
#define ZIGBEE_SECONDARY_CHANNEL_MASK 0x00000000

#define WATCHDOG_INTERVAL_MS      (60UL * 60UL * 1000UL)  /* 1 hour       */
#define WATCHDOG_PING_TIMEOUT_MS  (30UL * 1000UL)          /* 30s response */
#define WATCHDOG_PING_RETRIES     2                         /* retries before offline */

// ============================================================================
// ZCL CLUSTER / ATTRIBUTE IDs
// ============================================================================

#define CLUSTER_BASIC              0x0000
#define CLUSTER_POWER_CONFIG       0x0001
#define CLUSTER_ILLUMINANCE        0x0400
#define CLUSTER_TEMP_MEASUREMENT   0x0402
#define CLUSTER_HUMIDITY           0x0405
#define CLUSTER_OCCUPANCY_SENSING  0x0406
#define CLUSTER_IAS_ZONE           0x0500
#define CLUSTER_PRIVATE_TUYA       0xEF00

#define ATTR_BASIC_MANUFACTURER_NAME  0x0004
#define ATTR_BASIC_MODEL_IDENTIFIER   0x0005

#define ATTR_ILLUMINANCE_MEASURED     0x0000
#define ATTR_TEMPERATURE_MEASURED     0x0000
#define ATTR_HUMIDITY_MEASURED        0x0000
#define ATTR_OCCUPANCY                0x0000
#define ATTR_BATTERY_PERCENT          0x0021

/*
 * Tuya EF00 fading-time datapoint.
 * Frame: seq(1) dp(1) datatype(1) len_hi(1) len_lo(1) data(4)
 * datatype 0x02 = value (uint32 big-endian, 4 bytes)
 */
#define TUYA_DP_FADING_TIME  0x66

// ============================================================================
// LOGGING MACROS
// ============================================================================

#if RAW_LOGS_MODE
  #define RAW_LOG(...)              printf(__VA_ARGS__)
  #define DEV_LOG(tag, fmt, ...)    ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#else
  #define RAW_LOG(...)              do {} while(0)
  #define DEV_LOG(tag, fmt, ...)    do {} while(0)
#endif

/* Always visible in both modes */
#define PROD_LOG(tag, fmt, ...)     ESP_LOGI(tag, fmt, ##__VA_ARGS__)

// ============================================================================
// SENSOR TYPE ENUM
// ============================================================================

typedef enum {
    SENSOR_UNKNOWN    = 0,
    SENSOR_ZG_204ZV   = 1,
    SENSOR_ZG_205Z_A  = 2,
    SENSOR_ZG_102Z    = 3,
    SENSOR_ZG_102ZA   = 4,
} sensor_type_t;

typedef struct {
    const char   *model_id;
    sensor_type_t type;
} sensor_model_def_t;

/*
 * Exact modelID strings as broadcast by each sensor over the air.
 * ZG-205Z/A broadcasts "CK-BL702-MWS-01(7016)" — confirmed from Zigbee2MQTT.
 */
static const sensor_model_def_t k_sensor_models[] = {
    {"ZG-204ZV",              SENSOR_ZG_204ZV },
    {"CK-BL702-MWS-01(7016)", SENSOR_ZG_205Z_A},
    {"ZG-102Z",               SENSOR_ZG_102Z  },
    {"ZG-102ZA",              SENSOR_ZG_102ZA },
};

// ============================================================================
// RUNTIME METADATA
// ============================================================================

typedef struct {
    sensor_type_t type;
    bool          model_known;
    char          model_id[32];
    bool          bound_once;
    bool          reporting_configured;
    bool          enroll_sent;
    bool          fade_sent;
    bool          ping_pending;     /* true = ping sent, waiting for response */
    uint32_t      ping_sent_ms;     /* tick count when ping was sent          */
    uint8_t       rejoin_count;
} sensor_runtime_meta_t;

// ============================================================================
// GLOBALS
// ============================================================================

hub_config_safe_t            g_config              = {0};
static sensor_runtime_meta_t g_meta[MAX_SENSORS]   = {0};

static bool pairing_active         = false;
static bool pairing_window_expired = false;
static bool network_formed         = false;
static bool formation_requested    = false;
static bool formation_task_started = false;
static bool zigbee_ready           = false;
static bool g_dirty                = false;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

static void print_sensor_summary(void);
esp_err_t   save_config(hub_config_t *config);
esp_err_t   load_config(hub_config_t *config);

// ============================================================================
// THREAD-SAFE CONFIG ACCESS
// ============================================================================

hub_config_t *lock_config(void)
{
    if (!g_config.mutex) return NULL;
    if (xSemaphoreTake(g_config.mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
        return &g_config.data;
    ESP_LOGW(TAG, "lock_config timeout");
    return NULL;
}

void unlock_config(void)
{
    if (g_config.mutex) xSemaphoreGive(g_config.mutex);
}

static void mark_dirty(void) { g_dirty = true; }

// ============================================================================
// MODEL LOOKUP HELPERS
// ============================================================================

static const sensor_model_def_t *find_model_def(const char *model_id)
{
    for (size_t i = 0; i < sizeof(k_sensor_models) / sizeof(k_sensor_models[0]); i++) {
        if (strcmp(k_sensor_models[i].model_id, model_id) == 0)
            return &k_sensor_models[i];
    }
    return NULL;
}

static int find_sensor_index_by_short(uint16_t short_addr)
{
    hub_config_t *c = lock_config();
    if (!c) return -1;
    for (int i = 0; i < c->sensor_count; i++) {
        if (c->sensors[i].short_addr == short_addr) {
            unlock_config();
            return i;
        }
    }
    unlock_config();
    return -1;
}

static void set_default_sensor_name(sensor_t *s, int idx)
{
    if (s->sensor_name[0] == '\0')
        snprintf(s->sensor_name, SENSOR_NAME_LEN, "Sensor_%d", idx + 1);
}

static void apply_model_to_sensor(int idx, const char *model_id)
{
    const sensor_model_def_t *def = find_model_def(model_id);
    if (!def) {
        g_meta[idx].type        = SENSOR_UNKNOWN;
        g_meta[idx].model_known = false;
        ESP_LOGW(TAG, "Sensor %d unknown modelID: '%s'", idx + 1, model_id);
        return;
    }
    strncpy(g_meta[idx].model_id, model_id, sizeof(g_meta[idx].model_id) - 1);
    g_meta[idx].model_id[sizeof(g_meta[idx].model_id) - 1] = '\0';
    g_meta[idx].type        = def->type;
    g_meta[idx].model_known = true;

    hub_config_t *c = lock_config();
    if (c) {
        c->sensors[idx].sensor_type = (uint8_t)def->type;
        PROD_LOG(TAG, "[JOIN] Sensor_%d | %-24s | IEEE=%s | Short=0x%04hx",
                 idx + 1, model_id,
                 c->sensors[idx].ieee_addr, c->sensors[idx].short_addr);
        unlock_config();
    }
    DEV_LOG(TAG, "Sensor %d model identified: %s (type=%d)", idx + 1, model_id, (int)def->type);
}

static void restore_meta_from_nvs(hub_config_t *config)
{
    for (int i = 0; i < config->sensor_count; i++) {
        sensor_type_t t = (sensor_type_t)config->sensors[i].sensor_type;
        g_meta[i].type = t;
        if (t != SENSOR_UNKNOWN) {
            g_meta[i].model_known = true;
            for (size_t j = 0; j < sizeof(k_sensor_models)/sizeof(k_sensor_models[0]); j++) {
                if (k_sensor_models[j].type == t) {
                    strncpy(g_meta[i].model_id, k_sensor_models[j].model_id,
                            sizeof(g_meta[i].model_id) - 1);
                    break;
                }
            }
        }
        /* Assume offline until they announce — they will send DEVICE_ANNCE on reconnect */
        config->sensors[i].online = false;
    }
}

// ============================================================================
// HUB AGGREGATE PRESENCE  (call with config lock HELD)
// ============================================================================

static void update_hub_presence_locked(hub_config_t *c)
{
    bool any_occupied = false;
    for (int j = 0; j < c->sensor_count; j++) {
        sensor_type_t t = (sensor_type_t)c->sensors[j].sensor_type;
        if ((t == SENSOR_ZG_204ZV || t == SENSOR_ZG_205Z_A)
            && c->sensors[j].presence
            && c->sensors[j].online)   /* ONLY count online sensors */
            any_occupied = true;
    }
    bool changed = (c->hub_status.occupied != any_occupied);
    if (changed) {
        c->hub_status.last_change = time(NULL);
        PROD_LOG(TAG, "[HUB] presence=%s", any_occupied ? "OCCUPIED" : "VACANT");
    }
    c->hub_status.occupied  = any_occupied;
    c->hub_status.timestamp = time(NULL);
}

// ============================================================================
// DISPLAY
// ============================================================================

static void print_banner(void)
{
    printf("\n\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║       INNOVATSII EMS - SENSOR HUB                           ║\n");
    printf("║       Strict Per-Model Build                                 ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    fflush(stdout);
}

static void print_sensor_summary(void)
{
    hub_config_t *c = lock_config();
    if (!c) return;

    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║ SENSOR REGISTRY (%d/%d)  Hub: %s\n",
           c->sensor_count, MAX_SENSORS,
           c->hub_status.occupied ? "OCCUPIED" : "VACANT");
    printf("║ Network: %s | Pairing: %s\n",
           network_formed ? "ACTIVE" : "FORMING",
           pairing_active ? "OPEN"   : "CLOSED");
    printf("╚══════════════════════════════════════════════════════════════╝\n");

    for (int i = 0; i < c->sensor_count; i++) {
        sensor_t     *s      = &c->sensors[i];
        sensor_type_t t      = (sensor_type_t)s->sensor_type;
        const char   *status = s->online ? "[ONLINE] " : "[OFFLINE]";
        char          batt_buf[16];

        if (s->battery_pct == 0 && !s->online)
            snprintf(batt_buf, sizeof(batt_buf), "N/A");
        else
            snprintf(batt_buf, sizeof(batt_buf), "%u%%", s->battery_pct);

        if (t == SENSOR_ZG_204ZV) {
            bool no_env = (s->temperature_cdeg == 0 && s->humidity_cpct == 0 && !s->online);
            if (no_env) {
                printf("  [%d] %-10s %s ZG-204ZV  | presence=%-3s  temp=N/A  hum=N/A"
                       "  lux=%u  batt=%s  tamper=%s\n",
                       i + 1, s->sensor_name, status,
                       s->presence ? "YES" : "NO",
                       s->illuminance_raw, batt_buf,
                       s->tamper ? "YES" : "NO");
            } else {
                printf("  [%d] %-10s %s ZG-204ZV  | presence=%-3s  temp=%.2f\xC2\xB0""C"
                       "  hum=%.2f%%  lux=%u  batt=%s  tamper=%s\n",
                       i + 1, s->sensor_name, status,
                       s->presence ? "YES" : "NO",
                       (double)s->temperature_cdeg / 100.0,
                       (double)s->humidity_cpct    / 100.0,
                       s->illuminance_raw, batt_buf,
                       s->tamper ? "YES" : "NO");
            }
        } else if (t == SENSOR_ZG_205Z_A) {
            printf("  [%d] %-10s %s ZG-205Z/A | presence=%-3s%s  lux=%u\n",
                   i + 1, s->sensor_name, status,
                   s->presence ? "YES" : "NO",
                   (!s->online && !s->presence) ? " (stale)" : "",
                   s->illuminance_raw);
        } else if (t == SENSOR_ZG_102Z || t == SENSOR_ZG_102ZA) {
            printf("  [%d] %-10s %s %-8s | contact=%-6s  batt=%s  tamper=%s  batt_low=%s\n",
                   i + 1, s->sensor_name, status,
                   t == SENSOR_ZG_102Z ? "ZG-102Z" : "ZG-102ZA",
                   s->contact_open ? "OPEN" : "CLOSED",
                   batt_buf,
                   s->tamper      ? "YES" : "NO",
                   s->battery_low ? "YES" : "NO");
        } else {
            printf("  [%d] %-10s %s UNKNOWN (type=%u)\n",
                   i + 1, s->sensor_name, status, s->sensor_type);
        }
    }
    unlock_config();
    fflush(stdout);
}

// ============================================================================
// NVS PERSISTENCE
// ============================================================================

esp_err_t save_config(hub_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    nvs_handle_t handle;
    esp_err_t err = nvs_open("sensor_hub", NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    nvs_set_u8(handle, "mode",            (uint8_t)config->mode);
    nvs_set_u8(handle, "sensor_count",    config->sensor_count);
    nvs_set_u8(handle, "pairing_active",  config->pairing_active   ? 1 : 0);
    nvs_set_u8(handle, "pairing_expired", pairing_window_expired   ? 1 : 0);
    nvs_set_u8(handle, "hub_occupied",    config->hub_status.occupied ? 1 : 0);

    for (int i = 0; i < config->sensor_count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "sensor_%d", i);
        nvs_set_blob(handle, key, &config->sensors[i], sizeof(sensor_t));
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

esp_err_t load_config(hub_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    memset(g_meta, 0, sizeof(g_meta));
    pairing_window_expired = false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open("sensor_hub", NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        config->mode            = MODE_PAIRING;
        config->sensor_count    = 0;
        config->pairing_active  = true;
        config->pairing_started = time(NULL);
        PROD_LOG(TAG, "NVS empty — fresh install, entering pairing mode");
        return ESP_OK;
    }
    if (err != ESP_OK) return err;

    uint8_t mode = MODE_PAIRING;
    nvs_get_u8(handle, "mode", &mode);
    config->mode = (hub_mode_t)mode;

    uint8_t pa = 1;
    nvs_get_u8(handle, "pairing_active", &pa);
    config->pairing_active = (pa != 0);

    uint8_t pe = 0;
    nvs_get_u8(handle, "pairing_expired", &pe);
    pairing_window_expired = (pe != 0);

    uint8_t hub_occ = 0;
    nvs_get_u8(handle, "hub_occupied", &hub_occ);
    config->hub_status.occupied = (hub_occ != 0);

    uint8_t sensor_count = 0;
    nvs_get_u8(handle, "sensor_count", &sensor_count);
    if (sensor_count > MAX_SENSORS) sensor_count = MAX_SENSORS;

    config->sensor_count = 0;
    for (int i = 0; i < sensor_count; i++) {
        char   key[16];
        size_t size = sizeof(sensor_t);
        snprintf(key, sizeof(key), "sensor_%d", i);
        if (nvs_get_blob(handle, key, &config->sensors[i], &size) == ESP_OK)
            config->sensor_count++;
    }

    nvs_close(handle);
    restore_meta_from_nvs(config);

    RAW_LOG("[RAW] load_config: sensor_count=%u pairing_expired=%d hub_occupied=%d\n",
            (unsigned)config->sensor_count, pairing_window_expired,
            (int)config->hub_status.occupied);
    return ESP_OK;
}

// ============================================================================
// SENSOR REGISTRY
// ============================================================================

static void register_or_update_joined_sensor(uint16_t short_addr, const char *ieee)
{
    hub_config_t *c = lock_config();
    if (!c) return;

    int idx = -1;
    for (int i = 0; i < c->sensor_count; i++) {
        if (strcmp(c->sensors[i].ieee_addr, ieee) == 0) { idx = i; break; }
    }

    if (idx < 0) {
        if (c->sensor_count >= MAX_SENSORS) {
            unlock_config();
            ESP_LOGW(TAG, "Sensor registry full (%d/%d)", MAX_SENSORS, MAX_SENSORS);
            return;
        }
        idx = c->sensor_count++;
        memset(&c->sensors[idx], 0, sizeof(sensor_t));
        memset(&g_meta[idx],     0, sizeof(sensor_runtime_meta_t));
        PROD_LOG(TAG, "DEVICE_ANNCE new sensor slot %d for IEEE %s", idx + 1, ieee);
    } else {
        PROD_LOG(TAG, "DEVICE_ANNCE Sensor_%d re-joined (IEEE %s)", idx + 1, ieee);
    }

    c->sensors[idx].short_addr = short_addr;
    strncpy(c->sensors[idx].ieee_addr, ieee, IEEE_ADDR_STR_LEN - 1);
    c->sensors[idx].ieee_addr[IEEE_ADDR_STR_LEN - 1] = '\0';
    c->sensors[idx].endpoint  = 1;
    c->sensors[idx].last_seen = time(NULL);
    c->sensors[idx].online    = true;
    set_default_sensor_name(&c->sensors[idx], idx);

    unlock_config();
    mark_dirty();
}

// ============================================================================
// TUYA EF00 — FADING TIME = 0
// ============================================================================

static void send_fade_time_zero(uint16_t short_addr, uint8_t ep)
{
    /*
     * Correct 9-byte Tuya EF00 frame:
     *   seq(1) + dp(1) + datatype(1) + len_hi(1) + len_lo(1) + data(4)
     * datatype 0x02 = Tuya "value" (uint32 big-endian)
     */
    static const uint8_t payload[] = {
        0x00,                /* seq_num              */
        TUYA_DP_FADING_TIME, /* dp = 0x66            */
        0x02,                /* datatype = value     */
        0x00, 0x04,          /* data_len = 4         */
        0x00, 0x00, 0x00, 0x00  /* value = 0         */
    };

    ezb_zcl_custom_cluster_cmd_t cmd = {0};
    cmd.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
    cmd.cmd_ctrl.dst_addr.u.short_addr = short_addr;
    cmd.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
    cmd.cmd_ctrl.dst_ep                = ep;
    cmd.cmd_ctrl.cluster_id            = CLUSTER_PRIVATE_TUYA;
    cmd.cmd_ctrl.fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd.cmd_ctrl.fc.dis_default_rsp    = 1;
    cmd.cmd_id                         = 0x00;
    cmd.data_length                    = sizeof(payload);
    cmd.data                           = (uint8_t *)payload;

    ezb_err_t ret = ezb_zcl_custom_cluster_cmd_req(&cmd);
    if (ret == EZB_ERR_NONE)
        DEV_LOG(TAG, "fading_time=0 send OK to 0x%04hx ep%u", short_addr, ep);
    else
        ESP_LOGW(TAG, "fading_time send failed 0x%04hx: 0x%04x", short_addr, ret);
}

/* Deferred wrapper — sends fade command after a 2s delay to let sensor finish init */
typedef struct {
    uint16_t short_addr;
    uint8_t  ep;
} fade_args_t;

static void deferred_fade_task(void *arg)
{
    fade_args_t *a = (fade_args_t *)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));  /* 2s delay — let sensor finish init */
    send_fade_time_zero(a->short_addr, a->ep);
    free(a);
    vTaskDelete(NULL);
}

// ============================================================================
// REPORTING CONFIGURATION
// ============================================================================

static void configure_reporting_for_model(uint16_t short_addr, uint8_t ep)
{
    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0 || g_meta[idx].reporting_configured) return;

    sensor_type_t type = g_meta[idx].type;

    ezb_zcl_config_report_cmd_t cmd = {0};
    cmd.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
    cmd.cmd_ctrl.dst_addr.u.short_addr = short_addr;
    cmd.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
    cmd.cmd_ctrl.dst_ep                = ep;
    cmd.cmd_ctrl.fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd.cmd_ctrl.fc.dis_default_rsp    = 1;
    cmd.payload.record_number          = 1;

    if (type == SENSOR_ZG_204ZV) {
        /* IAS Zone — ZoneStatus attr 0x0002: heartbeat ≤2min, immediate on change */
        ezb_zcl_config_report_record_t ias = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = 0x0002, /* ZoneStatus */
            .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT16,
                          .min_interval = 2, .max_interval = 120,
                          .reportable_change = {.u16 = 1}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_IAS_ZONE;
        cmd.payload.record_field = &ias;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Reporting configured: IAS ZoneStatus for 0x%04hx", short_addr);

        /* Temperature: on change only (0.5°C threshold) */
        ezb_zcl_config_report_record_t temp = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_TEMPERATURE_MEASURED,
            .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_INT16,
                          .min_interval = 0, .max_interval = 0xFFFF,
                          .reportable_change = {.s16 = 50}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_TEMP_MEASUREMENT;
        cmd.payload.record_field = &temp;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Reporting configured: temperature for 0x%04hx", short_addr);

        /* Humidity: on change only (1% threshold) */
        ezb_zcl_config_report_record_t hum = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_HUMIDITY_MEASURED,
            .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT16,
                          .min_interval = 0, .max_interval = 0xFFFF,
                          .reportable_change = {.u16 = 100}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_HUMIDITY;
        cmd.payload.record_field = &hum;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Reporting configured: humidity for 0x%04hx", short_addr);

        /* Illuminance: on change only */
        ezb_zcl_config_report_record_t lux = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_ILLUMINANCE_MEASURED,
            .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT16,
                          .min_interval = 0, .max_interval = 0xFFFF,
                          .reportable_change = {.u16 = 500}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_ILLUMINANCE;
        cmd.payload.record_field = &lux;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Reporting configured: illuminance for 0x%04hx", short_addr);

        /* Battery: periodic + on change */
        ezb_zcl_config_report_record_t batt = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_BATTERY_PERCENT,
            .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT8,
                          .min_interval = 60, .max_interval = 3600,
                          .reportable_change = {.u8 = 2}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_POWER_CONFIG;
        cmd.payload.record_field = &batt;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Reporting configured: battery for 0x%04hx", short_addr);

    } else if (type == SENSOR_ZG_205Z_A) {
        /* Occupancy: force every 2 seconds */
        ezb_zcl_config_report_record_t occ = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_OCCUPANCY,
            .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT8,
                          .min_interval = 2, .max_interval = 2,
                          .reportable_change = {.u8 = 1}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_OCCUPANCY_SENSING;
        cmd.payload.record_field = &occ;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Reporting configured: occupancy for 0x%04hx", short_addr);

        /* Illuminance: on change only */
        ezb_zcl_config_report_record_t lux = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_ILLUMINANCE_MEASURED,
            .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT16,
                          .min_interval = 0, .max_interval = 0xFFFF,
                          .reportable_change = {.u16 = 500}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_ILLUMINANCE;
        cmd.payload.record_field = &lux;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Reporting configured: illuminance for 0x%04hx", short_addr);

    } else if (type == SENSOR_ZG_102Z || type == SENSOR_ZG_102ZA) {
        /* Battery: periodic + on change; IAS is event-driven, no config needed */
        ezb_zcl_config_report_record_t batt = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_BATTERY_PERCENT,
            .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT8,
                          .min_interval = 60, .max_interval = 3600,
                          .reportable_change = {.u8 = 2}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_POWER_CONFIG;
        cmd.payload.record_field = &batt;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Reporting configured: battery for 0x%04hx", short_addr);
    }

    g_meta[idx].reporting_configured = true;
}

// ============================================================================
// CLUSTER BINDING  (stack-allocated — no malloc/free race)
// ============================================================================

static void bind_model_clusters(uint16_t short_addr, const ezb_extaddr_t *ieee, uint8_t ep)
{
    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0 || !ieee || g_meta[idx].bound_once) return;

    ezb_extaddr_t coordinator_ieee;
    ezb_get_extended_address(&coordinator_ieee);

    static const uint16_t zg204_clusters[] = {
        CLUSTER_IAS_ZONE,
        CLUSTER_TEMP_MEASUREMENT,
        CLUSTER_HUMIDITY,
        CLUSTER_ILLUMINANCE,
        CLUSTER_POWER_CONFIG,
        CLUSTER_PRIVATE_TUYA
    };
    static const uint16_t zg205_clusters[] = {
        CLUSTER_IAS_ZONE,
        CLUSTER_OCCUPANCY_SENSING,
        CLUSTER_ILLUMINANCE,
        CLUSTER_PRIVATE_TUYA
    };
    static const uint16_t zg102_clusters[] = {
        CLUSTER_IAS_ZONE,
        CLUSTER_POWER_CONFIG
    };

    const uint16_t *clusters     = NULL;
    size_t          cluster_count = 0;

    switch (g_meta[idx].type) {
    case SENSOR_ZG_204ZV:
        clusters      = zg204_clusters;
        cluster_count = sizeof(zg204_clusters) / sizeof(zg204_clusters[0]);
        break;
    case SENSOR_ZG_205Z_A:
        clusters      = zg205_clusters;
        cluster_count = sizeof(zg205_clusters) / sizeof(zg205_clusters[0]);
        break;
    case SENSOR_ZG_102Z:
    case SENSOR_ZG_102ZA:
        clusters      = zg102_clusters;
        cluster_count = sizeof(zg102_clusters) / sizeof(zg102_clusters[0]);
        break;
    default:
        ESP_LOGW(TAG, "bind_model_clusters: unknown type for 0x%04hx", short_addr);
        return;
    }

    for (size_t i = 0; i < cluster_count; i++) {
        ezb_zdo_bind_req_t req = {0};
        req.dst_nwk_addr                 = short_addr;
        req.field.src_addr               = *ieee;
        req.field.src_ep                 = ep;
        req.field.cluster_id             = clusters[i];
        req.field.dst_addr_mode          = EZB_ADDR_MODE_EXT;
        req.field.dst_addr.extended_addr = coordinator_ieee;
        req.field.dst_ep                 = COORDINATOR_ENDPOINT;

        DEV_LOG(TAG, "Bind request 0x%04hx ep%u cluster 0x%04hx", short_addr, ep, clusters[i]);
        ezb_zdo_bind_req(&req);
    }

    g_meta[idx].bound_once = true;
}

// ============================================================================
// BASIC CLUSTER READ — MODEL IDENTIFICATION
// ============================================================================

static void request_model_id(uint16_t short_addr, uint8_t ep)
{
    uint16_t attrs[] = {ATTR_BASIC_MANUFACTURER_NAME, ATTR_BASIC_MODEL_IDENTIFIER};
    ezb_zcl_read_attr_cmd_t cmd = {
        .cmd_ctrl = {
            .dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT,
            .dst_addr.u.short_addr = short_addr,
            .src_ep                = COORDINATOR_ENDPOINT,
            .dst_ep                = ep,
            .cluster_id            = CLUSTER_BASIC,
            .fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV,
        },
        .payload.attr_number = 2,
        .payload.attr_field  = attrs,
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t ret = ezb_zcl_read_attr_cmd_req(&cmd);
    esp_zigbee_lock_release();

    if (ret == EZB_ERR_NONE)
        DEV_LOG(TAG, "modelId read sent to 0x%04hx ep%u", short_addr, ep);
    else
        DEV_LOG(TAG, "modelId read failed 0x%04hx ep%u: 0x%04x", short_addr, ep, ret);
}

// ============================================================================
// ZCL CALLBACKS
// ============================================================================

/* Mark a sensor as online; called from every callback that receives sensor data */
static void mark_sensor_online(int idx)
{
    hub_config_t *c = lock_config();
    if (!c) return;
    bool was_offline = !c->sensors[idx].online;
    c->sensors[idx].online    = true;
    c->sensors[idx].last_seen = time(NULL);
    if (idx < MAX_SENSORS) {
        g_meta[idx].ping_pending = false; /* clear any pending ping */
    }
    if (was_offline) {
        PROD_LOG(TAG, "[WDG] Sensor_%d back ONLINE", idx + 1);
        update_hub_presence_locked(c);
    }
    unlock_config();
}

/* Send a Basic cluster read (attr 0x0000) as a watchdog ping */
static void send_ping(uint16_t short_addr, uint8_t ep, int sensor_idx)
{
    (void)sensor_idx;
    uint16_t attr = 0x0000; /* ZCL version attribute */
    ezb_zcl_read_attr_cmd_t cmd = {
        .cmd_ctrl = {
            .dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT,
            .dst_addr.u.short_addr = short_addr,
            .src_ep                = COORDINATOR_ENDPOINT,
            .dst_ep                = ep,
            .cluster_id            = CLUSTER_BASIC,
            .fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV,
        },
        .payload.attr_number = 1,
        .payload.attr_field  = &attr,
    };
    esp_zigbee_lock_acquire(portMAX_DELAY);
    (void)ezb_zcl_read_attr_cmd_req(&cmd);
    esp_zigbee_lock_release();
}

static void zcl_core_read_attr_rsp_handler(ezb_zcl_cmd_read_attr_rsp_message_t *message)
{
    if (!message || !message->in.header) return;
    if (message->info.cluster_id != CLUSTER_BASIC) return;

    uint16_t short_addr = message->in.header->src_addr.u.short_addr;
    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0) return;

    mark_sensor_online(idx);

    uint8_t ep = message->in.header->src_ep;
    ezb_zcl_read_attr_rsp_variable_t *var = message->in.variables;
    while (var) {
        if (var->status == EZB_ZCL_STATUS_SUCCESS) {
            if (var->attr_id == 0x0000) {
                /* Watchdog ping response — ZCL version attribute */
                g_meta[idx].ping_pending = false;
            } else if (var->attr_id == ATTR_BASIC_MANUFACTURER_NAME) {
                uint8_t len = *(uint8_t *)var->attr_value;
                DEV_LOG(TAG, "Sensor %d manufacturer: %.*s",
                        idx + 1, len, (char *)(var->attr_value + 1));
            } else if (var->attr_id == ATTR_BASIC_MODEL_IDENTIFIER) {
                uint8_t len = *(uint8_t *)var->attr_value;
                char model[32] = {0};
                if (len >= sizeof(model)) len = (uint8_t)(sizeof(model) - 1);
                memcpy(model, (char *)(var->attr_value + 1), len);
                model[len] = '\0';
                apply_model_to_sensor(idx, model);
            }
        }
        var = var->next;
    }

    if (g_meta[idx].model_known) {
        ezb_extaddr_t sensor_ieee;
        if (ezb_address_extended_by_short(short_addr, &sensor_ieee) == EZB_ERR_NONE) {
            bind_model_clusters(short_addr, &sensor_ieee, ep);
            configure_reporting_for_model(short_addr, ep);
            sensor_type_t t = g_meta[idx].type;
            if ((t == SENSOR_ZG_204ZV || t == SENSOR_ZG_205Z_A) && !g_meta[idx].fade_sent) {
                fade_args_t *fa = malloc(sizeof(fade_args_t));
                if (fa) {
                    fa->short_addr = short_addr;
                    fa->ep         = ep;
                    if (xTaskCreate(deferred_fade_task, "fade_send", 2048, fa, 3, NULL) != pdPASS) {
                        free(fa); /* task creation failed — avoid leak */
                    }
                }
                g_meta[idx].fade_sent = true;
            }
        }
    }
}

/*
 * IAS Zone Enroll Request handler.
 *
 * The sensor sends this when it wants to enroll with the coordinator as its CIE.
 * We MUST respond with ezb_zcl_ias_zone_enroll_cmd_resp() using
 * ezb_zcl_ias_zone_enroll_rsp_cmd_t which uses ezb_zcl_cluster_cmd_ctrl_t.
 *
 * ezb_zcl_cluster_cmd_ctrl_t members (from zcl_common.h):
 *   dst_addr, dst_ep, src_ep, dis_default_rsp, cnf_ctx
 * NOTE: NO cluster_id, NO fc struct — those are in ezb_zcl_cmd_ctrl_t only.
 * The cluster_id is implicit in the function called.
 */
static void zcl_ias_zone_enroll_handler(ezb_zcl_ias_zone_enroll_req_message_t *message)
{
    if (!message || !message->in.header) return;

    uint16_t short_addr = message->in.header->src_addr.u.short_addr;
    uint8_t  src_ep     = message->in.header->src_ep;
    int      idx        = find_sensor_index_by_short(short_addr);
    uint8_t  zone_id    = (idx >= 0) ? (uint8_t)idx : 0;

    if (idx >= 0) mark_sensor_online(idx);

    DEV_LOG(TAG, "IAS EnrollReq from 0x%04hx ep%u zone_type=0x%04hx — replying zone_id=%u",
            short_addr, src_ep, message->in.payload.zone_type, zone_id);

    /*
     * ezb_zcl_ias_zone_enroll_rsp_cmd_t uses ezb_zcl_cluster_cmd_ctrl_t:
     *   - NO cluster_id field  (implicit — function handles it)
     *   - NO fc struct         (use dis_default_rsp bool directly)
     */
    ezb_zcl_ias_zone_enroll_rsp_cmd_t rsp = {0};
    rsp.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
    rsp.cmd_ctrl.dst_addr.u.short_addr = short_addr;
    rsp.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
    rsp.cmd_ctrl.dst_ep                = src_ep;
    rsp.cmd_ctrl.dis_default_rsp       = true;
    rsp.payload.enroll_rsp_code        = EZB_ZCL_IAS_ZONE_ENROLL_RESPONSE_CODE_SUCCESS;
    rsp.payload.zone_id                = zone_id;

    /* Correct function name from SDK: ezb_zcl_ias_zone_enroll_cmd_resp() */
    ezb_err_t ret = ezb_zcl_ias_zone_enroll_cmd_resp(&rsp);
    if (ret == EZB_ERR_NONE) {
        if (idx >= 0) g_meta[idx].enroll_sent = true;
        DEV_LOG(TAG, "Sent IAS enrollRsp to 0x%04hx zone_id=%u", short_addr, zone_id);
    } else {
        ESP_LOGW(TAG, "IAS enrollRsp failed 0x%04hx: 0x%04x", short_addr, ret);
    }
}

/* IAS Zone Status Change Notification — main event path for ALL IAS sensors */
static void zcl_ias_zone_status_change_handler(
        ezb_zcl_ias_zone_status_change_notif_message_t *message)
{
    if (!message || !message->in.header) return;

    uint16_t short_addr  = message->in.header->src_addr.u.short_addr;
    uint16_t zone_status = message->in.payload.zone_status;
    bool alarm1   = (zone_status & 0x0001) != 0;
    bool tamper   = (zone_status & 0x0004) != 0;
    bool batt_low = (zone_status & 0x0008) != 0;

    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0) {
        ESP_LOGW(TAG, "IAS notify from unknown device 0x%04hx zone_status=0x%04hx",
                 short_addr, zone_status);
        return;
    }

    RAW_LOG("[RAW] IAS zone_status=0x%04hx src=0x%04hx\n", zone_status, short_addr);

    mark_sensor_online(idx);

    hub_config_t *c = lock_config();
    if (!c) return;

    sensor_t     *s = &c->sensors[idx];
    sensor_type_t t = (sensor_type_t)s->sensor_type;
    s->last_seen    = time(NULL);
    s->tamper       = tamper;
    s->battery_low  = batt_low;

    if (t == SENSOR_ZG_204ZV || t == SENSOR_ZG_205Z_A) {
        if (s->presence != alarm1) s->last_change = time(NULL);
        s->presence = alarm1;
        /* Determine hub presence string for log */
        bool any_occ = false;
        for (int j = 0; j < c->sensor_count; j++) {
            sensor_type_t tj = (sensor_type_t)c->sensors[j].sensor_type;
            if ((tj == SENSOR_ZG_204ZV || tj == SENSOR_ZG_205Z_A)
                && c->sensors[j].presence && c->sensors[j].online)
                any_occ = true;
        }
        PROD_LOG(TAG, "[DATA] Sensor_%d [%s] presence=%s \xe2\x86\x92 Hub: %s",
                 idx + 1, g_meta[idx].model_id,
                 alarm1  ? "YES" : "NO",
                 any_occ ? "OCCUPIED" : "VACANT");
        update_hub_presence_locked(c);
    } else if (t == SENSOR_ZG_102Z || t == SENSOR_ZG_102ZA) {
        if (s->contact_open != alarm1) s->last_change = time(NULL);
        s->contact_open = alarm1;
        PROD_LOG(TAG, "[DATA] Sensor_%d [%s] contact=%s",
                 idx + 1, g_meta[idx].model_id,
                 alarm1 ? "OPEN" : "CLOSED");
    } else {
        ESP_LOGW(TAG, "IAS notify from unclassified sensor %d (type=%u)",
                 idx + 1, s->sensor_type);
    }

    unlock_config();
    mark_dirty();
}

/* Standard ZCL attribute report handler */
static void zcl_core_cmd_report_attr_handler(ezb_zcl_cmd_report_attr_message_t *message)
{
    if (!message || !message->in.header || !message->in.variables) return;

    uint16_t short_addr = message->in.header->src_addr.u.short_addr;
    uint16_t cluster_id = message->info.cluster_id;

    RAW_LOG("[RAW] ReportAttr src=0x%04hx cluster=0x%04hx\n", short_addr, cluster_id);

    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0) return;

    mark_sensor_online(idx);

    hub_config_t *c = lock_config();
    if (!c) return;

    sensor_t     *s       = &c->sensors[idx];
    sensor_type_t t       = (sensor_type_t)s->sensor_type;
    bool          changed = false;

    /* ---- ZG-204ZV ---- */
    if (t == SENSOR_ZG_204ZV) {
        if (cluster_id == CLUSTER_TEMP_MEASUREMENT) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables; v; v = v->next) {
                if (v->attr_id == ATTR_TEMPERATURE_MEASURED) {
                    int16_t raw = *(int16_t *)v->attr_value;
                    s->temperature_cdeg = raw;
                    s->last_seen = time(NULL);
                    changed = true;
                    PROD_LOG(TAG, "[DATA] Sensor_%d temp=%.2f\xC2\xB0""C",
                             idx + 1, (double)raw / 100.0);
                }
            }
        } else if (cluster_id == CLUSTER_HUMIDITY) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables; v; v = v->next) {
                if (v->attr_id == ATTR_HUMIDITY_MEASURED) {
                    uint16_t raw = *(uint16_t *)v->attr_value;
                    s->humidity_cpct = raw;
                    s->last_seen = time(NULL);
                    changed = true;
                    PROD_LOG(TAG, "[DATA] Sensor_%d humidity=%.2f%%",
                             idx + 1, (double)raw / 100.0);
                }
            }
        } else if (cluster_id == CLUSTER_ILLUMINANCE) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables; v; v = v->next) {
                if (v->attr_id == ATTR_ILLUMINANCE_MEASURED) {
                    s->illuminance_raw = *(uint16_t *)v->attr_value;
                    s->last_seen = time(NULL);
                    changed = true;
                    PROD_LOG(TAG, "[DATA] Sensor_%d illuminance=%u (ZCL raw)",
                             idx + 1, s->illuminance_raw);
                }
            }
        } else if (cluster_id == CLUSTER_POWER_CONFIG) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables; v; v = v->next) {
                if (v->attr_id == ATTR_BATTERY_PERCENT) {
                    /* BatteryPercentageRemaining unit = 0.5% — divide by 2 */
                    uint8_t pct = (*(uint8_t *)v->attr_value) / 2;
                    s->battery_pct = pct;
                    s->last_seen   = time(NULL);
                    changed = true;
                    PROD_LOG(TAG, "[DATA] Sensor_%d battery=%u%%", idx + 1, pct);
                }
            }
        }
    }

    /* ---- ZG-205Z/A ---- */
    else if (t == SENSOR_ZG_205Z_A) {
        if (cluster_id == CLUSTER_OCCUPANCY_SENSING) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables; v; v = v->next) {
                if (v->attr_id == ATTR_OCCUPANCY) {
                    bool occ = (*(uint8_t *)v->attr_value) != 0;
                    if (s->presence != occ) s->last_change = time(NULL);
                    s->presence  = occ;
                    s->last_seen = time(NULL);
                    changed = true;
                    /* Determine hub presence for log */
                    bool any_occ = false;
                    for (int j = 0; j < c->sensor_count; j++) {
                        sensor_type_t tj = (sensor_type_t)c->sensors[j].sensor_type;
                        if ((tj == SENSOR_ZG_204ZV || tj == SENSOR_ZG_205Z_A)
                            && c->sensors[j].presence && c->sensors[j].online)
                            any_occ = true;
                    }
                    PROD_LOG(TAG, "[DATA] Sensor_%d [%s] presence=%s \xe2\x86\x92 Hub: %s",
                             idx + 1, g_meta[idx].model_id,
                             occ     ? "YES"      : "NO",
                             any_occ ? "OCCUPIED" : "VACANT");
                    update_hub_presence_locked(c);
                }
            }
        } else if (cluster_id == CLUSTER_ILLUMINANCE) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables; v; v = v->next) {
                if (v->attr_id == ATTR_ILLUMINANCE_MEASURED) {
                    s->illuminance_raw = *(uint16_t *)v->attr_value;
                    s->last_seen = time(NULL);
                    changed = true;
                    PROD_LOG(TAG, "[DATA] Sensor_%d illuminance=%u (ZCL raw)",
                             idx + 1, s->illuminance_raw);
                }
            }
        }
    }

    /* ---- ZG-102Z / ZG-102ZA ---- */
    else if (t == SENSOR_ZG_102Z || t == SENSOR_ZG_102ZA) {
        /* Door state comes via IAS Status Change Notification.
           Only battery attribute reports arrive here. */
        if (cluster_id == CLUSTER_POWER_CONFIG) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables; v; v = v->next) {
                if (v->attr_id == ATTR_BATTERY_PERCENT) {
                    uint8_t pct = (*(uint8_t *)v->attr_value) / 2;
                    s->battery_pct = pct;
                    s->last_seen   = time(NULL);
                    changed = true;
                    PROD_LOG(TAG, "[DATA] Sensor_%d battery=%u%%", idx + 1, pct);
                }
            }
        }
    }

    unlock_config();
    if (changed) mark_dirty();
}

static void zcl_core_cmd_report_config_rsp_handler(
        ezb_zcl_cmd_config_report_rsp_message_t *message)
{
    if (!message) return;
    DEV_LOG(TAG, "ConfigReportRsp ep(%d) cluster(0x%04x) status(0x%02x)",
            message->info.dst_ep, message->info.cluster_id, message->info.status);
}

static void zcl_core_cmd_default_rsp_handler(ezb_zcl_cmd_default_rsp_message_t *message)
{
    if (!message) return;
    DEV_LOG(TAG, "ZCL DefaultRsp status=0x%02x", message->in.status_code);
}

/* Master ZCL action dispatcher */
static void esp_zigbee_zcl_core_action_handler(
        ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {

    case EZB_ZCL_CORE_READ_ATTR_RSP_CB_ID:
        zcl_core_read_attr_rsp_handler(
            (ezb_zcl_cmd_read_attr_rsp_message_t *)message);
        break;

    case EZB_ZCL_CORE_REPORT_ATTR_CB_ID:
        zcl_core_cmd_report_attr_handler(
            (ezb_zcl_cmd_report_attr_message_t *)message);
        break;

    case EZB_ZCL_CORE_CONFIG_REPORT_RSP_CB_ID:
        zcl_core_cmd_report_config_rsp_handler(
            (ezb_zcl_cmd_config_report_rsp_message_t *)message);
        break;

    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID:
        zcl_core_cmd_default_rsp_handler(
            (ezb_zcl_cmd_default_rsp_message_t *)message);
        break;

    /* IAS Zone enroll request — MUST respond or sensor stays silent forever */
    case EZB_ZCL_CORE_IAS_ZONE_ENROLL_CB_ID:
        zcl_ias_zone_enroll_handler(
            (ezb_zcl_ias_zone_enroll_req_message_t *)message);
        break;

    /* IAS Zone status change — main event for ALL IAS sensors */
    case EZB_ZCL_CORE_IAS_ZONE_STATUS_CHANGE_NOTIF_CB_ID:
        zcl_ias_zone_status_change_handler(
            (ezb_zcl_ias_zone_status_change_notif_message_t *)message);
        break;

    default:
        RAW_LOG("[RAW] ZCL action 0x%04lx (unhandled)\n", (unsigned long)callback_id);
        break;
    }
}

// ============================================================================
// RAW FRAME HANDLER — debug logging ONLY, no data parsing
// ============================================================================

static bool raw_frame_handler(const ezb_zcl_raw_frame_t *raw_frame)
{
    if (!raw_frame || !raw_frame->header) return false;

    uint16_t src_addr   = raw_frame->header->src_addr.u.short_addr;
    uint16_t cluster_id = raw_frame->header->cluster_id;

    RAW_LOG("[RAW] frame src=0x%04hx cluster=0x%04hx len=%u\n",
            src_addr, cluster_id, (unsigned)raw_frame->payload_length);

    if (cluster_id == CLUSTER_PRIVATE_TUYA && raw_frame->payload_length > 0) {
        RAW_LOG("[RAW] EF00:");
        for (uint16_t i = 0; i < raw_frame->payload_length; i++)
            RAW_LOG(" %02X", raw_frame->payload[i]);
        RAW_LOG("\n");
    }

    return false; /* let SDK continue normal processing */
}

// ============================================================================
// ZIGBEE APP SIGNAL HANDLER — pairing/joining (preserved exactly)
// ============================================================================

static void deferred_formation_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    if (!formation_requested) {
        formation_requested = true;
        RAW_LOG("[RAW] starting network formation\n");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION);
    }
    vTaskDelete(NULL);
}

static void pairing_window_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(PAIRING_DURATION_SEC * 1000));
    pairing_window_expired = true;
    pairing_active         = false;
    ezb_bdb_open_network(0);
    PROD_LOG(TAG, "Pairing window closed");
    print_sensor_summary();
    vTaskDelete(NULL);
}

static void active_ep_callback(const ezb_zdo_active_ep_req_result_t *result, void *user_ctx)
{
    uint16_t short_addr = (uint16_t)(uintptr_t)user_ctx;
    if (!result || result->error != EZB_ERR_NONE || !result->rsp) return;
    DEV_LOG(TAG, "Active EPs for 0x%04hx: count=%u", short_addr, result->rsp->active_ep_count);
    for (uint8_t i = 0; i < result->rsp->active_ep_count; i++)
        request_model_id(short_addr, result->rsp->active_ep_list[i]);
}

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    switch (ezb_app_signal_get_type(app_signal)) {

    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        if (!formation_task_started) {
            formation_task_started = true;
            xTaskCreate(deferred_formation_task, "zb_form", 3072, NULL, 5, NULL);
        }
        break;

    case EZB_BDB_SIGNAL_FORMATION: {
        ezb_bdb_comm_status_t status =
            *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            network_formed         = true;
            zigbee_ready           = true;
            pairing_window_expired = false;
            pairing_active         = true;
            PROD_LOG(TAG, "Network formed — opening pairing for %ds", PAIRING_DURATION_SEC);
            ezb_bdb_open_network((uint8_t)PAIRING_DURATION_SEC);
            xTaskCreate(pairing_window_task, "pair_win", 3072, NULL, 4, NULL);
        } else {
            ESP_LOGW(TAG, "Network formation failed (status=%d) — will retry", (int)status);
            formation_requested = false;
        }
        break;
    }

    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        const ezb_zdo_signal_device_annce_params_t *ann =
            ezb_app_signal_get_params(app_signal);
        if (!ann) break;

        char ieee_str[IEEE_ADDR_STR_LEN] = {0};
        snprintf(ieee_str, sizeof(ieee_str),
                 "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 ann->device_addr.u8[7], ann->device_addr.u8[6],
                 ann->device_addr.u8[5], ann->device_addr.u8[4],
                 ann->device_addr.u8[3], ann->device_addr.u8[2],
                 ann->device_addr.u8[1], ann->device_addr.u8[0]);

        PROD_LOG(TAG, "DEVICE_ANNCE short=0x%04hx IEEE=%s", ann->short_addr, ieee_str);
        register_or_update_joined_sensor(ann->short_addr, ieee_str);

        ezb_zdo_active_ep_req_t req = {0};
        req.dst_nwk_addr               = ann->short_addr;
        req.field.nwk_addr_of_interest = ann->short_addr;
        req.cb                         = active_ep_callback;
        req.user_ctx                   = (void *)(uintptr_t)ann->short_addr;
        ezb_zdo_active_ep_req(&req);
        break;
    }

    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t duration = *(uint8_t *)ezb_app_signal_get_params(app_signal);
        pairing_active = (duration != 0);
        if (duration == 0 && !pairing_window_expired) {
            pairing_window_expired = true;
            print_sensor_summary();
        }
        break;
    }

    default:
        break;
    }
    return true;
}

// ============================================================================
// COORDINATOR DEVICE SETUP
// ============================================================================

static esp_err_t esp_zigbee_create_coordinator_device(void)
{
    static const char manufacturer_name[] = "Innovatsii EMS";
    static const char model_identifier[]  = "sensor-hub-zigbee";

    ezb_af_device_desc_t            dev_desc    = ezb_af_create_device_desc();
    ezb_zha_custom_gateway_config_t gateway_cfg = EZB_ZHA_CUSTOM_GATEWAY_CONFIG();
    ezb_af_ep_desc_t                ep_desc     =
        ezb_zha_create_custom_gateway(COORDINATOR_ENDPOINT, &gateway_cfg);

    ezb_zcl_cluster_desc_t basic_desc =
        ezb_af_endpoint_get_cluster_desc(ep_desc, EZB_ZCL_CLUSTER_ID_BASIC,
                                         EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc,
        EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)manufacturer_name);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc,
        EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  (void *)model_identifier);

    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc,
        ezb_zcl_on_off_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER)));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc,
        ezb_zcl_occupancy_sensing_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER)));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));

    ezb_zcl_raw_command_handler_register(raw_frame_handler);
    ezb_zcl_core_action_handler_register(esp_zigbee_zcl_core_action_handler);

    return ESP_OK;
}

static esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));
    return ESP_OK;
}

// ============================================================================
// PERSIST TASK
// ============================================================================

static void persist_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (g_dirty) {
            hub_config_t *c = lock_config();
            if (c) {
                esp_err_t err = save_config(c);
                unlock_config();
                if (err == ESP_OK)
                    g_dirty = false;
                else
                    ESP_LOGW(TAG, "NVS save failed: %s", esp_err_to_name(err));
            }
        }
    }
}

// ============================================================================
// WATCHDOG TASK  (only active when WATCHDOG_ENABLE == 1)
// ============================================================================

#if WATCHDOG_ENABLE
static void sensor_watchdog_task(void *arg)
{
    (void)arg;
    /* Stagger first check — wait one full interval before first ping */
    vTaskDelay(pdMS_TO_TICKS(WATCHDOG_INTERVAL_MS));

    for (;;) {
        hub_config_t *c = lock_config();
        int count = c ? c->sensor_count : 0;
        unlock_config();

        PROD_LOG(TAG, "[WDG] Starting health check (%d sensors)", count);

        for (int i = 0; i < count; i++) {
            hub_config_t *cfg = lock_config();
            if (!cfg) continue;
            uint16_t short_addr      = cfg->sensors[i].short_addr;
            uint8_t  ep              = cfg->sensors[i].endpoint;
            bool     currently_online = cfg->sensors[i].online;
            unlock_config();

            if (!currently_online) continue; /* already offline, skip */

            /* Send Basic cluster read (attr 0x0000 = ZCL version) as ping */
            g_meta[i].ping_pending  = true;
            g_meta[i].ping_sent_ms  = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            g_meta[i].rejoin_count  = 0;

            send_ping(short_addr, ep, i);

            /* Wait for response and retry up to WATCHDOG_PING_RETRIES times */
            vTaskDelay(pdMS_TO_TICKS(WATCHDOG_PING_TIMEOUT_MS));
            while (g_meta[i].ping_pending && g_meta[i].rejoin_count < WATCHDOG_PING_RETRIES) {
                g_meta[i].rejoin_count++;
                send_ping(short_addr, ep, i);
                vTaskDelay(pdMS_TO_TICKS(WATCHDOG_PING_TIMEOUT_MS));
            }

            if (g_meta[i].ping_pending) {
                /* Still no response — mark offline */
                hub_config_t *cc = lock_config();
                if (cc) {
                    cc->sensors[i].online = false;
                    PROD_LOG(TAG, "[WDG] Sensor_%d OFFLINE \xe2\x80\x94 cleared from presence",
                             i + 1);
                    update_hub_presence_locked(cc);
                    unlock_config();
                    mark_dirty();
                }
            } else {
                PROD_LOG(TAG, "[WDG] Sensor_%d ONLINE", i + 1);
            }

            /* Small gap between pings to avoid flooding */
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* Wait for next cycle */
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_INTERVAL_MS));
    }
}
#endif /* WATCHDOG_ENABLE */

// ============================================================================
// ZIGBEE STACK TASK
// ============================================================================

static void esp_zigbee_stack_main_task(void *pvParameters)
{
    (void)pvParameters;

    esp_zigbee_config_t zigbee_config = {0};
    zigbee_config.device_config.device_type              = EZB_NWK_DEVICE_TYPE_COORDINATOR;
    zigbee_config.device_config.install_code_policy      = false;
    zigbee_config.device_config.zczr_config.max_children = MAX_SENSORS;
    zigbee_config.platform_config.storage_partition_name =
        ESP_ZIGBEE_STORAGE_PARTITION_NAME;
    zigbee_config.platform_config.radio_config.radio_mode =
        ESP_ZIGBEE_RADIO_MODE_NATIVE;

    ESP_ERROR_CHECK(esp_zigbee_init(&zigbee_config));
    ESP_ERROR_CHECK(esp_zigbee_setup_commissioning());
    ESP_ERROR_CHECK(esp_zigbee_create_coordinator_device());
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    ESP_ERROR_CHECK(esp_zigbee_launch_mainloop());

    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

// ============================================================================
// APP ENTRY POINT
// ============================================================================

void app_main(void)
{
    print_banner();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME));

#if FACTORY_RESET_MODE
    ESP_LOGW(TAG, "FACTORY RESET MODE — erasing NVS");
    nvs_flash_erase();
    nvs_flash_init();
#endif

    g_config.mutex = xSemaphoreCreateMutex();
    if (!g_config.mutex) {
        ESP_LOGE(TAG, "Failed to create config mutex — halting");
        return;
    }

    memset(&g_config.data, 0, sizeof(g_config.data));
    load_config(&g_config.data);

    xTaskCreate(esp_zigbee_stack_main_task, "Zigbee_main", 4096 * 2, NULL, 5, NULL);
    xTaskCreate(persist_task,               "persist_task", 2048,    NULL, 3, NULL);
#if WATCHDOG_ENABLE
    xTaskCreate(sensor_watchdog_task, "watchdog", 3072, NULL, 2, NULL);
#endif
}