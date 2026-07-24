/*
 * main.c — Sensor Hub Zigbee Coordinator
 * Innovatsii EMS — Pico 1
 *
 * Confirmed sensor modelID strings (from live Zigbee capture):
 *   ZG-204ZV              — HOBEIAN mmWave: IAS presence + Temp + Hum + Batt
 *   CK-BL702-MWS-01(7016) — HOBEIAN mmWave: IAS presence + Occupancy
 *   ZG-102Z               — HOBEIAN door:   IAS contact + Tamper + Batt
 *   ZG-102ZA              — HOBEIAN door:   IAS contact + Tamper + Batt
 *
 * Phase 2 — complete integration:
 *   UART Master layer fully integrated
 *   Unit occupancy state machine (VACANT/OCCUPIED)
 *   Door alarm via uart_master_notify_door_state()
 *   Heartbeat, hub_boot, hub_ready notifications
 *   Battery reporting to Master
 *   Watchdog with configurable runtime intervals
 *   All configurable parameters from Master via set_config
 *   IAS Zone CLIENT cluster — fixes door sensor enroll loop
 *   Illuminance discarded — reduces Zigbee traffic
 *   Battery clamped to 100%
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
#include "esp_timer.h"
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
#include "uart_master.h"
#include "zigbee_gateway.h"

// ============================================================================
// COMPILE-TIME FLAGS
// ============================================================================

#ifndef ESP_ZIGBEE_STORAGE_PARTITION_NAME
#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "zb_storage"
#endif

#define TAG "SENSOR_HUB"

/*
 * FACTORY_RESET_MODE
 *   1 = erase sensor NVS on every boot (lab testing / fresh deployment)
 *   0 = normal operation (sensors persist across reboot)
 *
 * NOTE: This ONLY erases the "nvs" partition (sensor registry + uart config).
 *       The "zb_storage" partition (Zigbee network keys) is NOT erased.
 *       Sensors that are already paired will re-join automatically without
 *       pressing any button because they still have valid network keys.
 *       Set to 1 when deploying a fresh unit to a customer so old test
 *       sensor names are wiped. Set back to 0 for production.
 *
 * RAW_LOGS_MODE
 *   1 = developer — shows all ZCL bytes, EF00 hex, bind ops, enroll
 *   0 = production — shows only JOIN/DATA/HUB/UNIT/WDG tagged events
 *
 * WATCHDOG_ENABLE
 *   0 = disabled (lab debug — avoids false offline during testing)
 *   1 = enabled  (production)
 */
#define FACTORY_RESET_MODE  1
#define RAW_LOGS_MODE       1
#define WATCHDOG_ENABLE     1

#define ZIGBEE_PRIMARY_CHANNEL_MASK   0x07FFF800UL
#define ZIGBEE_SECONDARY_CHANNEL_MASK 0x00000000UL

/* Watchdog retry count — number of ping retries before marking offline */
#define WATCHDOG_PING_RETRIES  2

// ============================================================================
// LOGGING MACROS
// ============================================================================

static void uptime_str(char *buf, size_t len)
{
    uint64_t sec = esp_timer_get_time() / 1000000ULL;
    snprintf(buf, len, "%02u:%02u:%02u",
             (unsigned)(sec / 3600),
             (unsigned)((sec % 3600) / 60),
             (unsigned)(sec % 60));
}

#define PROD_LOG(tag, fmt, ...) do {                          \
    char _ts[10]; uptime_str(_ts, sizeof(_ts));               \
    ESP_LOGI(tag, "[%s] " fmt, _ts, ##__VA_ARGS__);           \
} while (0)

#if RAW_LOGS_MODE
  #define RAW_LOG(...)           printf(__VA_ARGS__)
  #define DEV_LOG(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#else
  #define RAW_LOG(...)           do {} while (0)
  #define DEV_LOG(tag, fmt, ...) do {} while (0)
#endif

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

#define ATTR_BASIC_ZCL_VERSION        0x0000
#define ATTR_BASIC_MANUFACTURER_NAME  0x0004
#define ATTR_BASIC_MODEL_IDENTIFIER   0x0005
#define ATTR_TEMPERATURE_MEASURED     0x0000
#define ATTR_HUMIDITY_MEASURED        0x0000
#define ATTR_OCCUPANCY                0x0000
#define ATTR_IAS_ZONE_STATUS          0x0002
#define ATTR_BATTERY_PERCENT          0x0021

#define TUYA_DP_FADING_TIME  0x66

// ============================================================================
// MODEL DEFINITIONS
// Role auto-assigned from model — no configuration needed.
// ============================================================================

typedef struct {
    const char   *model_id;
    sensor_type_t type;
    sensor_role_t role;
} sensor_model_def_t;

/*
 * Exact modelID strings as broadcast over the air (confirmed from live capture).
 * ZG-205Z/A broadcasts "CK-BL702-MWS-01(7016)" — not the product name.
 */
static const sensor_model_def_t k_sensor_models[] = {
    {"ZG-204ZV",              SENSOR_ZG_204ZV,  ROLE_PRESENCE},
    {"CK-BL702-MWS-01(7016)", SENSOR_ZG_205Z_A, ROLE_PRESENCE},
    {"ZG-102Z",               SENSOR_ZG_102Z,   ROLE_DOOR    },
    {"ZG-102ZA",              SENSOR_ZG_102ZA,  ROLE_DOOR    },
};

static const char *k_friendly[] = {
    "UNKNOWN", "ZG-204ZV", "ZG-205Z/A", "ZG-102Z", "ZG-102ZA",
};

const char *friendly_name_from_type(sensor_type_t t)
{
    if ((unsigned)t < sizeof(k_friendly) / sizeof(k_friendly[0]))
        return k_friendly[(unsigned)t];
    return "UNKNOWN";
}

const char *unit_state_str(unit_occupancy_t s)
{
    return s == UNIT_OCCUPIED ? "OCCUPIED" : "VACANT";
}

// ============================================================================
// RUNTIME METADATA (not persisted — rebuilt from NVS sensor_type on boot)
// ============================================================================

typedef struct {
    bool    model_known;
    char    model_id[32];
    bool    bound_once;
    bool    reporting_configured;
    bool    enroll_sent;
    bool    fade_sent;
    bool    ping_pending;
    uint8_t rejoin_count;
} sensor_runtime_meta_t;

// ============================================================================
// GLOBALS
// ============================================================================

hub_config_safe_t            g_config            = {0};
static sensor_runtime_meta_t g_meta[MAX_SENSORS] = {0};

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
static void update_hub_presence_locked(hub_config_t *c);
static void evaluate_unit_occupancy_locked(hub_config_t *c);

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

void mark_dirty(void) { g_dirty = true; }

// ============================================================================
// HELPERS
// ============================================================================

static const sensor_model_def_t *find_model_def(const char *model_id)
{
    for (size_t i = 0;
         i < sizeof(k_sensor_models) / sizeof(k_sensor_models[0]); i++) {
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
        g_meta[idx].model_known = false;
        ESP_LOGW(TAG, "Sensor %d unknown modelID: '%s'", idx + 1, model_id);
        return;
    }

    strncpy(g_meta[idx].model_id, model_id,
            sizeof(g_meta[idx].model_id) - 1);
    g_meta[idx].model_id[sizeof(g_meta[idx].model_id) - 1] = '\0';
    g_meta[idx].model_known = true;

    hub_config_t *c = lock_config();
    if (c) {
        c->sensors[idx].sensor_type = (uint8_t)def->type;
        c->sensors[idx].sensor_role = (uint8_t)def->role;
        PROD_LOG(TAG, "[JOIN] Sensor_%d | %-10s | IEEE=%s | Short=0x%04hx | Role=%s",
                 idx + 1,
                 friendly_name_from_type(def->type),
                 c->sensors[idx].ieee_addr,
                 c->sensors[idx].short_addr,
                 def->role == ROLE_DOOR ? "DOOR" : "PRESENCE");
        unlock_config();
    }
}

static void restore_meta_from_nvs(hub_config_t *config)
{
    for (int i = 0; i < config->sensor_count; i++) {
        sensor_type_t t = (sensor_type_t)config->sensors[i].sensor_type;
        /* All sensors start OFFLINE on boot — must re-announce to become active */
        config->sensors[i].online = false;
        if (t != SENSOR_UNKNOWN) {
            g_meta[i].model_known = true;
            for (size_t j = 0;
                 j < sizeof(k_sensor_models) / sizeof(k_sensor_models[0]); j++) {
                if (k_sensor_models[j].type == t) {
                    strncpy(g_meta[i].model_id, k_sensor_models[j].model_id,
                            sizeof(g_meta[i].model_id) - 1);
                    break;
                }
            }
        }
    }
}

// ============================================================================
// UNIT OCCUPANCY ENGINE  (call with config lock HELD)
//
// Triggered ONLY by door OPEN→CLOSED event (sets door_closed_pending).
// Re-evaluated when presence changes while door_closed_pending is true.
//
// Rules:
//   VACANT  → OCCUPIED : any online ROLE_PRESENCE sensor = YES
//   OCCUPIED → VACANT  : all online ROLE_PRESENCE sensors = NO
//
// Presence alone never changes unit state.
// Sensor going offline never changes unit state (safe default).
// ============================================================================

static void evaluate_unit_occupancy_locked(hub_config_t *c)
{
    if (!c->door_closed_pending) return;

    bool any_yes       = false;
    bool all_no        = true;
    int  online_count  = 0;

    /* Fixed buffer snapshot for log — no allocation */
    char snap[64] = {0};
    int  spos = 0;
    spos += snprintf(snap + spos, sizeof(snap) - (size_t)spos, "[");

    for (int i = 0; i < c->sensor_count; i++) {
        if ((sensor_role_t)c->sensors[i].sensor_role != ROLE_PRESENCE) continue;
        if (!c->sensors[i].online) continue;
        online_count++;
        bool p = c->sensors[i].presence;
        if (p) { any_yes = true; all_no = false; }
        int n = snprintf(snap + spos, sizeof(snap) - (size_t)spos,
                         "S%d:%s ", i + 1, p ? "YES" : "NO");
        if (n > 0 && (spos + n) < (int)sizeof(snap) - 2) spos += n;
    }
    if (spos > 1 && snap[spos - 1] == ' ') spos--;
    snprintf(snap + spos, sizeof(snap) - (size_t)spos, "]");

    c->door_closed_pending = false;

    if (online_count == 0) {
        PROD_LOG(TAG, "[UNIT] %s (no online presence sensors)",
                 unit_state_str(c->unit_state));
        return;
    }

    unit_occupancy_t new_state = c->unit_state;
    if      (c->unit_state == UNIT_VACANT   && any_yes) new_state = UNIT_OCCUPIED;
    else if (c->unit_state == UNIT_OCCUPIED && all_no)  new_state = UNIT_VACANT;

    if (new_state != c->unit_state) {
        c->unit_state         = new_state;
        c->unit_state_changed = time(NULL);
        mark_dirty();
        PROD_LOG(TAG, "[UNIT] %s | presence=%s",
                 unit_state_str(new_state), snap);
        uart_master_send_unit_occupancy(unit_state_str(new_state));
    } else {
        PROD_LOG(TAG, "[UNIT] %s (no change) | presence=%s",
                 unit_state_str(c->unit_state), snap);
    }
}

// ============================================================================
// HUB AGGREGATE PRESENCE  (call with config lock HELD)
// Only counts online sensors — offline sensors excluded.
// ============================================================================

static void update_hub_presence_locked(hub_config_t *c)
{
    bool any_occupied = false;
    for (int j = 0; j < c->sensor_count; j++) {
        sensor_type_t t = (sensor_type_t)c->sensors[j].sensor_type;
        if ((t == SENSOR_ZG_204ZV || t == SENSOR_ZG_205Z_A)
            && c->sensors[j].presence
            && c->sensors[j].online)
            any_occupied = true;
    }
    bool changed = (c->hub_status.occupied != any_occupied);
    if (changed) {
        c->hub_status.last_change = time(NULL);
        PROD_LOG(TAG, "[HUB] presence=%s",
                 any_occupied ? "OCCUPIED" : "VACANT");
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
    printf("║       Pico 1 — Phase 2                                       ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    fflush(stdout);
}

static void print_sensor_summary(void)
{
    hub_config_t *c = lock_config();
    if (!c) return;

    char ts[10];
    uptime_str(ts, sizeof(ts));

    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║ [%s] SENSOR REGISTRY (%d/%d)  Hub: %-8s  Unit: %-8s\n",
           ts,
           c->sensor_count, MAX_SENSORS,
           c->hub_status.occupied ? "OCCUPIED" : "VACANT",
           unit_state_str(c->unit_state));
    printf("║ Network: %s | Pairing: %s | Watchdog: %s\n",
           network_formed  ? "ACTIVE"  : "FORMING",
           pairing_active  ? "OPEN"    : "CLOSED",
           WATCHDOG_ENABLE ? "ENABLED" : "DISABLED");
    printf("╚══════════════════════════════════════════════════════════════╝\n");

    for (int i = 0; i < c->sensor_count; i++) {
        sensor_t     *s       = &c->sensors[i];
        sensor_type_t t       = (sensor_type_t)s->sensor_type;
        sensor_role_t role    = (sensor_role_t)s->sensor_role;
        const char   *status  = s->online ? "[ONLINE] " : "[OFFLINE]";
        const char   *rolestr = role == ROLE_DOOR     ? "DOOR    " :
                                role == ROLE_PRESENCE ? "PRESENCE" :
                                                        "UNKNOWN ";

        printf("  [%d] %-20s %s %-10s %-8s IEEE=%-23s Short=0x%04hx\n",
               i + 1, s->sensor_name, status,
               friendly_name_from_type(t), rolestr,
               s->ieee_addr, s->short_addr);

        if (t == SENSOR_ZG_204ZV) {
            printf("        presence=%-3s  temp=%.1f°C  hum=%.1f%%"
                   "  batt=%u%%  tamper=%s%s\n",
                   s->presence ? "YES" : "NO",
                   (double)s->temperature_cdeg / 100.0,
                   (double)s->humidity_cpct    / 100.0,
                   s->battery_pct,
                   s->tamper ? "YES" : "NO",
                   s->online ? "" : "  [stale]");
        } else if (t == SENSOR_ZG_205Z_A) {
            printf("        presence=%-3s  tamper=%s%s\n",
                   s->presence ? "YES" : "NO",
                   s->tamper ? "YES" : "NO",
                   s->online ? "" : "  [stale]");
        } else if (t == SENSOR_ZG_102Z || t == SENSOR_ZG_102ZA) {
            printf("        contact=%-6s  batt=%u%%  tamper=%s"
                   "  batt_low=%s%s\n",
                   s->contact_open ? "OPEN" : "CLOSED",
                   s->battery_pct,
                   s->tamper      ? "YES" : "NO",
                   s->battery_low ? "YES" : "NO",
                   s->online ? "" : "  [stale]");
        } else {
            printf("        UNKNOWN (type=%u)\n", s->sensor_type);
        }
    }
    printf("\n  Unit Occupancy : %s\n", unit_state_str(c->unit_state));
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
    nvs_set_u8(handle, "pairing_active",  config->pairing_active         ? 1 : 0);
    nvs_set_u8(handle, "pairing_expired", pairing_window_expired          ? 1 : 0);
    nvs_set_u8(handle, "hub_occupied",    config->hub_status.occupied     ? 1 : 0);
    nvs_set_u8(handle, "unit_state",      (uint8_t)config->unit_state);

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
        config->mode                = MODE_PAIRING;
        config->sensor_count        = 0;
        config->pairing_active      = true;
        config->pairing_started     = time(NULL);
        config->unit_state          = UNIT_VACANT;
        config->door_closed_pending = false;
        PROD_LOG(TAG, "NVS empty — fresh install, unit=VACANT, pairing=OPEN");
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

    uint8_t unit_st = UNIT_VACANT;
    nvs_get_u8(handle, "unit_state", &unit_st);
    config->unit_state          = (unit_occupancy_t)unit_st;
    config->door_closed_pending = false; /* never carry pending across reboot */

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

    RAW_LOG("[RAW] load_config: sensor_count=%u pairing_expired=%d "
            "hub_occupied=%d unit_state=%s\n",
            (unsigned)config->sensor_count, pairing_window_expired,
            (int)config->hub_status.occupied,
            unit_state_str(config->unit_state));
    return ESP_OK;
}

// ============================================================================
// SENSOR REGISTRY
// ============================================================================

static void register_or_update_joined_sensor(uint16_t short_addr,
                                              const char *ieee)
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
            ESP_LOGW(TAG, "Sensor registry full (%d/%d)",
                     MAX_SENSORS, MAX_SENSORS);
            return;
        }
        idx = c->sensor_count++;
        memset(&c->sensors[idx], 0, sizeof(sensor_t));
        memset(&g_meta[idx],     0, sizeof(sensor_runtime_meta_t));
        DEV_LOG(TAG, "New sensor slot %d for IEEE %s", idx + 1, ieee);
    } else {
        DEV_LOG(TAG, "Known sensor %d re-joined (IEEE %s)", idx + 1, ieee);
    }

    c->sensors[idx].short_addr = short_addr;
    strncpy(c->sensors[idx].ieee_addr, ieee, IEEE_ADDR_STR_LEN - 1);
    c->sensors[idx].ieee_addr[IEEE_ADDR_STR_LEN - 1] = '\0';
    c->sensors[idx].endpoint   = 1;
    c->sensors[idx].last_seen  = time(NULL);
    c->sensors[idx].online     = true;
    g_meta[idx].ping_pending   = false;
    set_default_sensor_name(&c->sensors[idx], idx);

    unlock_config();
    mark_dirty();
}

// ============================================================================
// TUYA EF00 — FADING TIME = 0
// ============================================================================

static void send_fade_time_zero(uint16_t short_addr, uint8_t ep)
{
    static const uint8_t payload[] = {
        0x00,
        TUYA_DP_FADING_TIME,
        0x02,
        0x00, 0x04,
        0x00, 0x00, 0x00, 0x00
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
    DEV_LOG(TAG, "fading_time=0 → 0x%04hx ep%u ret=0x%04x",
            short_addr, ep, ret);
}

typedef struct { uint16_t short_addr; uint8_t ep; } fade_args_t;

static void deferred_fade_task(void *arg)
{
    fade_args_t *a = (fade_args_t *)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));
    send_fade_time_zero(a->short_addr, a->ep);
    free(a);
    vTaskDelete(NULL);
}

// ============================================================================
// REPORTING CONFIGURATION
// Illuminance: NOT configured — discarded silently on receipt.
// ============================================================================

static void configure_reporting_for_model(uint16_t short_addr, uint8_t ep)
{
    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0 || g_meta[idx].reporting_configured) return;

    /* Read type from persisted config — NOT from g_meta */
    hub_config_t *cfg_tmp = lock_config();
    if (!cfg_tmp) return;
    sensor_type_t type = (sensor_type_t)cfg_tmp->sensors[idx].sensor_type;
    unlock_config();

    ezb_zcl_config_report_cmd_t cmd = {0};
    cmd.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
    cmd.cmd_ctrl.dst_addr.u.short_addr = short_addr;
    cmd.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
    cmd.cmd_ctrl.dst_ep                = ep;
    cmd.cmd_ctrl.fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd.cmd_ctrl.fc.dis_default_rsp    = 1;
    cmd.payload.record_number          = 1;

    if (type == SENSOR_ZG_204ZV) {

        /* IAS ZoneStatus — 2s heartbeat + immediate on change */
        ezb_zcl_config_report_record_t ias = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_IAS_ZONE_STATUS,
            .client    = {.attr_type       = EZB_ZCL_ATTR_TYPE_UINT16,
                          .min_interval    = 2,
                          .max_interval    = 120,
                          .reportable_change = {.u16 = 1}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_IAS_ZONE;
        cmd.payload.record_field = &ias;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Configured IAS reporting for 0x%04hx", short_addr);

        /* Temperature — on change (0.5°C threshold) */
        ezb_zcl_config_report_record_t temp = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_TEMPERATURE_MEASURED,
            .client    = {.attr_type       = EZB_ZCL_ATTR_TYPE_INT16,
                          .min_interval    = 0,
                          .max_interval    = 0xFFFF,
                          .reportable_change = {.s16 = 50}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_TEMP_MEASUREMENT;
        cmd.payload.record_field = &temp;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Configured temp reporting for 0x%04hx", short_addr);

        /* Humidity — on change (1% threshold) */
        ezb_zcl_config_report_record_t hum = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_HUMIDITY_MEASURED,
            .client    = {.attr_type       = EZB_ZCL_ATTR_TYPE_UINT16,
                          .min_interval    = 0,
                          .max_interval    = 0xFFFF,
                          .reportable_change = {.u16 = 100}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_HUMIDITY;
        cmd.payload.record_field = &hum;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Configured humidity reporting for 0x%04hx", short_addr);

        /* Battery — periodic + on change */
        ezb_zcl_config_report_record_t batt = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_BATTERY_PERCENT,
            .client    = {.attr_type       = EZB_ZCL_ATTR_TYPE_UINT8,
                          .min_interval    = 60,
                          .max_interval    = 3600,
                          .reportable_change = {.u8 = 2}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_POWER_CONFIG;
        cmd.payload.record_field = &batt;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Configured battery reporting for 0x%04hx", short_addr);

    } else if (type == SENSOR_ZG_205Z_A) {

        /* Occupancy — every 2 seconds (presence heartbeat) */
        ezb_zcl_config_report_record_t occ = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_OCCUPANCY,
            .client    = {.attr_type       = EZB_ZCL_ATTR_TYPE_UINT8,
                          .min_interval    = 2,
                          .max_interval    = 2,
                          .reportable_change = {.u8 = 1}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_OCCUPANCY_SENSING;
        cmd.payload.record_field = &occ;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Configured occupancy reporting (2s) for 0x%04hx",
                short_addr);

    } else if (type == SENSOR_ZG_102Z || type == SENSOR_ZG_102ZA) {

        /* Battery — door state is event-driven via IAS */
        ezb_zcl_config_report_record_t batt = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_BATTERY_PERCENT,
            .client    = {.attr_type       = EZB_ZCL_ATTR_TYPE_UINT8,
                          .min_interval    = 60,
                          .max_interval    = 3600,
                          .reportable_change = {.u8 = 2}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_POWER_CONFIG;
        cmd.payload.record_field = &batt;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        DEV_LOG(TAG, "Configured battery reporting for 0x%04hx", short_addr);
    }

    g_meta[idx].reporting_configured = true;
}

// ============================================================================
// CLUSTER BINDING
// Illuminance removed from all bind lists — not needed.
// ============================================================================

static void bind_model_clusters(uint16_t short_addr,
                                 const ezb_extaddr_t *ieee, uint8_t ep)
{
    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0 || !ieee || g_meta[idx].bound_once) return;

    ezb_extaddr_t coordinator_ieee;
    ezb_get_extended_address(&coordinator_ieee);

    static const uint16_t zg204_clusters[] = {
        CLUSTER_IAS_ZONE, CLUSTER_TEMP_MEASUREMENT,
        CLUSTER_HUMIDITY, CLUSTER_POWER_CONFIG, CLUSTER_PRIVATE_TUYA
    };
    static const uint16_t zg205_clusters[] = {
        CLUSTER_IAS_ZONE, CLUSTER_OCCUPANCY_SENSING, CLUSTER_PRIVATE_TUYA
    };
    static const uint16_t zg102_clusters[] = {
        CLUSTER_IAS_ZONE, CLUSTER_POWER_CONFIG
    };

    const uint16_t *clusters      = NULL;
    size_t          cluster_count = 0;

    hub_config_t *c = lock_config();
    sensor_type_t t = c ? (sensor_type_t)c->sensors[idx].sensor_type
                        : SENSOR_UNKNOWN;
    if (c) unlock_config();

    switch (t) {
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
        ESP_LOGW(TAG, "bind_model_clusters: unknown type 0x%04hx", short_addr);
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
        DEV_LOG(TAG, "Bind 0x%04hx ep%u cluster 0x%04hx",
                short_addr, ep, clusters[i]);
        ezb_zdo_bind_req(&req);
    }

    g_meta[idx].bound_once = true;
}

// ============================================================================
// WATCHDOG PING
// ============================================================================

static void send_ping(uint16_t short_addr, uint8_t ep)
{
    uint16_t attr = ATTR_BASIC_ZCL_VERSION;
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

// ============================================================================
// BASIC CLUSTER READ — MODEL IDENTIFICATION
// ============================================================================

static void request_model_id(uint16_t short_addr, uint8_t ep)
{
    uint16_t attrs[] = {ATTR_BASIC_MANUFACTURER_NAME,
                        ATTR_BASIC_MODEL_IDENTIFIER};
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
        DEV_LOG(TAG, "Sent modelId read to 0x%04hx ep%u", short_addr, ep);
    else
        ESP_LOGW(TAG, "modelId read failed 0x%04hx ep%u: 0x%04x",
                 short_addr, ep, ret);
}

// ============================================================================
// ZCL CALLBACKS
// ============================================================================

static void zcl_core_read_attr_rsp_handler(
        ezb_zcl_cmd_read_attr_rsp_message_t *message)
{
    if (!message || !message->in.header) return;
    if (message->info.cluster_id != CLUSTER_BASIC) return;

    uint16_t short_addr = message->in.header->src_addr.u.short_addr;
    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0) return;

    /* Any response = sensor is alive — mark online inside lock */
    {
        hub_config_t *c = lock_config();
        if (c) {
            bool was_offline = !c->sensors[idx].online;
            c->sensors[idx].online    = true;
            c->sensors[idx].last_seen = time(NULL);
            g_meta[idx].ping_pending  = false;
            if (was_offline) {
                PROD_LOG(TAG, "[WDG] %s back ONLINE",
                         c->sensors[idx].sensor_name);
                uart_master_send_sensor_health(
                    c->sensors[idx].sensor_name, true);
                update_hub_presence_locked(c);
            }
            unlock_config();
        }
    }

    bool model_read = false;
    ezb_zcl_read_attr_rsp_variable_t *var = message->in.variables;
    while (var) {
        if (var->status == EZB_ZCL_STATUS_SUCCESS) {
            if (var->attr_id == ATTR_BASIC_ZCL_VERSION) {
                DEV_LOG(TAG, "Ping response from 0x%04hx", short_addr);
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
                model_read = true;
            }
        }
        var = var->next;
    }

    if (model_read && g_meta[idx].model_known) {
        uint8_t ep = message->in.header->src_ep;
        ezb_extaddr_t sensor_ieee;
        if (ezb_address_extended_by_short(short_addr, &sensor_ieee)
            == EZB_ERR_NONE) {
            bind_model_clusters(short_addr, &sensor_ieee, ep);
            configure_reporting_for_model(short_addr, ep);

            hub_config_t *c = lock_config();
            sensor_type_t t = c
                ? (sensor_type_t)c->sensors[idx].sensor_type
                : SENSOR_UNKNOWN;
            if (c) unlock_config();

            if ((t == SENSOR_ZG_204ZV || t == SENSOR_ZG_205Z_A)
                && !g_meta[idx].fade_sent) {
                fade_args_t *fa = malloc(sizeof(fade_args_t));
                if (fa) {
                    fa->short_addr = short_addr;
                    fa->ep         = ep;
                    xTaskCreate(deferred_fade_task, "fade",
                                2048, fa, 3, NULL);
                }
                g_meta[idx].fade_sent = true;
            }
        }
    }
}

/*
 * IAS Zone Enroll Request.
 * Must respond IMMEDIATELY — before any other processing.
 * Delayed response causes sensor to leave and re-join in a loop.
 * IAS Zone CLIENT cluster must be registered on coordinator endpoint.
 */
static void zcl_ias_zone_enroll_handler(
        ezb_zcl_ias_zone_enroll_req_message_t *message)
{
    if (!message || !message->in.header) return;

    uint16_t short_addr = message->in.header->src_addr.u.short_addr;
    uint8_t  src_ep     = message->in.header->src_ep;
    int      idx        = find_sensor_index_by_short(short_addr);
    uint8_t  zone_id    = (idx >= 0) ? (uint8_t)idx : 0;

    ezb_zcl_ias_zone_enroll_rsp_cmd_t rsp = {0};
    rsp.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
    rsp.cmd_ctrl.dst_addr.u.short_addr = short_addr;
    rsp.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
    rsp.cmd_ctrl.dst_ep                = src_ep;
    rsp.cmd_ctrl.dis_default_rsp       = true;
    rsp.payload.enroll_rsp_code        =
        EZB_ZCL_IAS_ZONE_ENROLL_RESPONSE_CODE_SUCCESS;
    rsp.payload.zone_id                = zone_id;

    ezb_err_t ret = ezb_zcl_ias_zone_enroll_cmd_resp(&rsp);
    if (ret == EZB_ERR_NONE) {
        if (idx >= 0) g_meta[idx].enroll_sent = true;
        DEV_LOG(TAG, "IAS enrollRsp → 0x%04hx zone_id=%u",
                short_addr, zone_id);
    } else {
        ESP_LOGW(TAG, "IAS enrollRsp FAILED 0x%04hx: 0x%04x",
                 short_addr, ret);
    }

    /* Mark online inside lock — no separate lock/unlock race */
    if (idx >= 0) {
        hub_config_t *c = lock_config();
        if (c) {
            bool was_offline = !c->sensors[idx].online;
            c->sensors[idx].online    = true;
            c->sensors[idx].last_seen = time(NULL);
            g_meta[idx].ping_pending  = false;
            if (was_offline) {
                PROD_LOG(TAG, "[WDG] %s back ONLINE",
                         c->sensors[idx].sensor_name);
                uart_master_send_sensor_health(
                    c->sensors[idx].sensor_name, true);
            }
            unlock_config();
        }
    }
}

/*
 * IAS Zone Status Change Notification.
 *
 * DOOR sensors (ROLE_DOOR):
 *   OPEN→CLOSED sets door_closed_pending and evaluates occupancy.
 *   All state changes sent to Master via UART.
 *   Door alarm tracker notified on every state change.
 *
 * PRESENCE sensors (ZG-204ZV via IAS):
 *   Only logs and notifies Master on state CHANGE — not on 2s heartbeat.
 *   If door_closed_pending, re-evaluates unit occupancy.
 *
 * All sensors: marked ONLINE inside the lock on every message.
 */
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
        ESP_LOGW(TAG,
                 "IAS notify from unknown device 0x%04hx zone_status=0x%04hx",
                 short_addr, zone_status);
        return;
    }

    RAW_LOG("[RAW] IAS zone_status=0x%04hx src=0x%04hx\n",
            zone_status, short_addr);

    hub_config_t *c = lock_config();
    if (!c) return;

    /* Mark online inside lock — eliminates double-lock race */
    bool was_offline = !c->sensors[idx].online;
    c->sensors[idx].online    = true;
    c->sensors[idx].last_seen = time(NULL);
    g_meta[idx].ping_pending  = false;
    if (was_offline) {
        PROD_LOG(TAG, "[WDG] %s back ONLINE", c->sensors[idx].sensor_name);
        uart_master_send_sensor_health(c->sensors[idx].sensor_name, true);
    }

    sensor_t     *s    = &c->sensors[idx];
    sensor_type_t t    = (sensor_type_t)s->sensor_type;
    sensor_role_t role = (sensor_role_t)s->sensor_role;
    s->tamper          = tamper;
    s->battery_low     = batt_low;

    if (role == ROLE_DOOR) {
        bool state_changed = (s->contact_open != alarm1);
        bool was_open      = s->contact_open;
        s->contact_open    = alarm1;

        if (state_changed) {
            s->last_change = time(NULL);
            PROD_LOG(TAG, "[DATA] %s [%s] contact=%s",
                     s->sensor_name, friendly_name_from_type(t),
                     alarm1 ? "OPEN" : "CLOSED");

            /* Send door event to Master */
            uart_master_send_door(s->sensor_name, alarm1);

            /* Notify door alarm tracker */
            uart_master_notify_door_state(idx, s->sensor_name, alarm1);

            /* OPEN→CLOSED: trigger occupancy evaluation */
            if (was_open && !alarm1) {
                c->door_closed_pending = true;
                evaluate_unit_occupancy_locked(c);
            }
        }

    } else if (role == ROLE_PRESENCE ||
               t == SENSOR_ZG_204ZV  ||
               t == SENSOR_ZG_205Z_A) {

        bool state_changed = (s->presence != alarm1);
        s->presence = alarm1;

        if (state_changed) {
            s->last_change = time(NULL);
            PROD_LOG(TAG, "[DATA] %s [%s] presence=%s",
                     s->sensor_name, friendly_name_from_type(t),
                     alarm1 ? "YES" : "NO");
            update_hub_presence_locked(c);
            uart_master_send_sensor_presence(s->sensor_name,
                                             friendly_name_from_type(t),
                                             alarm1);
        }

        if (c->door_closed_pending)
            evaluate_unit_occupancy_locked(c);

    } else {
        ESP_LOGW(TAG,
                 "IAS notify from unclassified sensor %d (type=%u role=%u)",
                 idx + 1, s->sensor_type, s->sensor_role);
    }

    unlock_config();
    mark_dirty();
}

/*
 * ZCL Attribute Report handler.
 * Illuminance: discarded immediately — too many updates, not needed.
 * Battery: reported to Master on every update.
 * All handlers: mark sensor online inside lock.
 */
static void zcl_core_cmd_report_attr_handler(
        ezb_zcl_cmd_report_attr_message_t *message)
{
    if (!message || !message->in.header || !message->in.variables) return;

    uint16_t short_addr = message->in.header->src_addr.u.short_addr;
    uint16_t cluster_id = message->info.cluster_id;

    RAW_LOG("[RAW] ReportAttr src=0x%04hx cluster=0x%04hx\n",
            short_addr, cluster_id);

    /* Discard illuminance — sensor sends regardless of no binding */
    if (cluster_id == CLUSTER_ILLUMINANCE) return;

    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0) return;

    hub_config_t *c = lock_config();
    if (!c) return;

    /* Mark online inside lock */
    bool was_offline = !c->sensors[idx].online;
    c->sensors[idx].online    = true;
    c->sensors[idx].last_seen = time(NULL);
    g_meta[idx].ping_pending  = false;
    if (was_offline) {
        PROD_LOG(TAG, "[WDG] %s back ONLINE", c->sensors[idx].sensor_name);
        uart_master_send_sensor_health(c->sensors[idx].sensor_name, true);
        update_hub_presence_locked(c);
    }

    sensor_t     *s       = &c->sensors[idx];
    sensor_type_t t       = (sensor_type_t)s->sensor_type;
    bool          changed = false;

    /* ---- ZG-204ZV ---- */
    if (t == SENSOR_ZG_204ZV) {

        if (cluster_id == CLUSTER_TEMP_MEASUREMENT) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables;
                 v; v = v->next) {
                if (v->attr_id == ATTR_TEMPERATURE_MEASURED) {
                    s->temperature_cdeg = *(int16_t *)v->attr_value;
                    changed = true;
                    PROD_LOG(TAG, "[DATA] %s temp=%.2f°C",
                             s->sensor_name,
                             (double)s->temperature_cdeg / 100.0);
                    uart_master_send_environment(
                        s->sensor_name,
                        (float)s->temperature_cdeg / 100.0f,
                        (float)s->humidity_cpct    / 100.0f);
                }
            }
        } else if (cluster_id == CLUSTER_HUMIDITY) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables;
                 v; v = v->next) {
                if (v->attr_id == ATTR_HUMIDITY_MEASURED) {
                    s->humidity_cpct = *(uint16_t *)v->attr_value;
                    changed = true;
                    PROD_LOG(TAG, "[DATA] %s humidity=%.2f%%",
                             s->sensor_name,
                             (double)s->humidity_cpct / 100.0);
                    uart_master_send_environment(
                        s->sensor_name,
                        (float)s->temperature_cdeg / 100.0f,
                        (float)s->humidity_cpct    / 100.0f);
                }
            }
        } else if (cluster_id == CLUSTER_POWER_CONFIG) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables;
                 v; v = v->next) {
                if (v->attr_id == ATTR_BATTERY_PERCENT) {
                    uint8_t raw = *(uint8_t *)v->attr_value;
                    uint8_t pct = raw / 2;
                    if (pct > 100) pct = 100;
                    s->battery_pct = pct;
                    changed = true;
                    PROD_LOG(TAG, "[DATA] %s battery=%u%%",
                             s->sensor_name, s->battery_pct);
                    uart_master_send_battery(s->sensor_name, s->battery_pct);
                }
            }
        }
    }

    /* ---- ZG-205Z/A ---- */
    else if (t == SENSOR_ZG_205Z_A) {
        if (cluster_id == CLUSTER_OCCUPANCY_SENSING) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables;
                 v; v = v->next) {
                if (v->attr_id == ATTR_OCCUPANCY) {
                    bool occ          = (*(uint8_t *)v->attr_value) != 0;
                    bool state_chgd   = (s->presence != occ);
                    s->presence = occ;
                    changed     = true;

                    if (state_chgd) {
                        s->last_change = time(NULL);
                        PROD_LOG(TAG, "[DATA] %s [ZG-205Z/A] presence=%s",
                                 s->sensor_name, occ ? "YES" : "NO");
                        update_hub_presence_locked(c);
                        uart_master_send_sensor_presence(
                            s->sensor_name,
                            friendly_name_from_type(t),
                            occ);
                    }
                    if (c->door_closed_pending)
                        evaluate_unit_occupancy_locked(c);
                }
            }
        }
    }

    /* ---- ZG-102Z / ZG-102ZA ---- */
    else if (t == SENSOR_ZG_102Z || t == SENSOR_ZG_102ZA) {
        if (cluster_id == CLUSTER_POWER_CONFIG) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables;
                 v; v = v->next) {
                if (v->attr_id == ATTR_BATTERY_PERCENT) {
                    uint8_t raw = *(uint8_t *)v->attr_value;
                    uint8_t pct = raw / 2;
                    if (pct > 100) pct = 100;
                    s->battery_pct = pct;
                    changed = true;
                    PROD_LOG(TAG, "[DATA] %s battery=%u%%",
                             s->sensor_name, s->battery_pct);
                    uart_master_send_battery(s->sensor_name, s->battery_pct);
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
            message->info.dst_ep,
            message->info.cluster_id,
            message->info.status);
}

static void zcl_core_cmd_default_rsp_handler(
        ezb_zcl_cmd_default_rsp_message_t *message)
{
    if (!message) return;
    DEV_LOG(TAG, "ZCL DefaultRsp status=0x%02x", message->in.status_code);
}

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
    case EZB_ZCL_CORE_IAS_ZONE_ENROLL_CB_ID:
        zcl_ias_zone_enroll_handler(
            (ezb_zcl_ias_zone_enroll_req_message_t *)message);
        break;
    case EZB_ZCL_CORE_IAS_ZONE_STATUS_CHANGE_NOTIF_CB_ID:
        zcl_ias_zone_status_change_handler(
            (ezb_zcl_ias_zone_status_change_notif_message_t *)message);
        break;
    default:
        RAW_LOG("[RAW] ZCL action 0x%04lx (unhandled)\n",
                (unsigned long)callback_id);
        break;
    }
}

// ============================================================================
// RAW FRAME HANDLER — logging only
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

    return false;
}

// ============================================================================
// WATCHDOG TASK
// Pings each sensor and marks offline after no response.
// Reads all intervals from uart_hub_config — fully runtime-configurable.
// ============================================================================

#if WATCHDOG_ENABLE
static void sensor_watchdog_task(void *arg)
{
    (void)arg;

    /* Wait one full cycle before first check — avoids false offline
     * during the boot + re-join period */
    uart_hub_config_t cfg;
    uart_master_get_config(&cfg);
    vTaskDelay(pdMS_TO_TICKS(
        (uint32_t)cfg.watchdog_interval_min * 60UL * 1000UL));

    for (;;) {
        uart_master_get_config(&cfg);
        uint32_t interval_ms = (uint32_t)cfg.watchdog_interval_min
                               * 60UL * 1000UL;
        uint32_t ping_ms     = (uint32_t)cfg.watchdog_ping_timeout_sec
                               * 1000UL;

        hub_config_t *cc = lock_config();
        int count = cc ? cc->sensor_count : 0;
        if (cc) unlock_config();

        PROD_LOG(TAG, "[WDG] Health check starting (%d sensors)", count);

        for (int i = 0; i < count; i++) {
            hub_config_t *c = lock_config();
            if (!c) continue;

            bool     online     = c->sensors[i].online;
            uint16_t short_addr = c->sensors[i].short_addr;
            uint8_t  ep         = c->sensors[i].endpoint;
            char     name[SENSOR_NAME_LEN];
            strncpy(name, c->sensors[i].sensor_name, SENSOR_NAME_LEN - 1);
            name[SENSOR_NAME_LEN - 1] = '\0';
            unlock_config();

            if (!online) {
                DEV_LOG(TAG, "[WDG] %s already OFFLINE — skip", name);
                continue;
            }

            g_meta[i].ping_pending = true;
            g_meta[i].rejoin_count = 0;
            send_ping(short_addr, ep);
            vTaskDelay(pdMS_TO_TICKS(ping_ms));

            if (g_meta[i].ping_pending) {
                g_meta[i].rejoin_count++;
                DEV_LOG(TAG, "[WDG] %s no response — retry %u/%u",
                        name, g_meta[i].rejoin_count, WATCHDOG_PING_RETRIES);
                send_ping(short_addr, ep);
                vTaskDelay(pdMS_TO_TICKS(ping_ms));
            }

            if (g_meta[i].ping_pending) {
                hub_config_t *co = lock_config();
                if (co) {
                    co->sensors[i].online = false;
                    PROD_LOG(TAG, "[WDG] %s OFFLINE", name);
                    update_hub_presence_locked(co);
                    unlock_config();
                    mark_dirty();
                }
                uart_master_send_sensor_health(name, false);
                g_meta[i].ping_pending = false;
            } else {
                PROD_LOG(TAG, "[WDG] %s ONLINE", name);
            }

            vTaskDelay(pdMS_TO_TICKS(500));
        }

        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
}
#endif /* WATCHDOG_ENABLE */

// ============================================================================
// ZIGBEE APP SIGNAL HANDLER
// ============================================================================

static void deferred_formation_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    if (!formation_requested) {
        formation_requested = true;
        RAW_LOG("[RAW] starting network formation\n");
        ezb_bdb_start_top_level_commissioning(
            EZB_BDB_MODE_NETWORK_FORMATION);
    }
    vTaskDelete(NULL);
}

static void pairing_window_task(void *arg)
{
    (void)arg;
    uart_hub_config_t cfg;
    uart_master_get_config(&cfg);
    vTaskDelay(pdMS_TO_TICKS(
        (uint32_t)cfg.pairing_duration_sec * 1000UL));
    pairing_window_expired = true;
    pairing_active         = false;
    ezb_bdb_open_network(0);
    PROD_LOG(TAG, "Pairing window closed");
    print_sensor_summary();

    /* Notify Master — all sensors that rejoined are now known */
    uart_master_send_hub_ready();

    vTaskDelete(NULL);
}

static void active_ep_callback(
        const ezb_zdo_active_ep_req_result_t *result, void *user_ctx)
{
    uint16_t short_addr = (uint16_t)(uintptr_t)user_ctx;
    if (!result || result->error != EZB_ERR_NONE || !result->rsp) return;
    DEV_LOG(TAG, "Active EPs for 0x%04hx: count=%u",
            short_addr, result->rsp->active_ep_count);
    for (uint8_t i = 0; i < result->rsp->active_ep_count; i++)
        request_model_id(short_addr, result->rsp->active_ep_list[i]);
}

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    switch (ezb_app_signal_get_type(app_signal)) {

    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        if (!formation_task_started) {
            formation_task_started = true;
            xTaskCreate(deferred_formation_task, "zb_form",
                        3072, NULL, 5, NULL);
        }
        break;

    case EZB_BDB_SIGNAL_FORMATION: {
        ezb_bdb_comm_status_t status =
            *((ezb_bdb_comm_status_t *)
              ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            network_formed         = true;
            zigbee_ready           = true;
            pairing_window_expired = false;
            pairing_active         = true;

            uart_hub_config_t cfg;
            uart_master_get_config(&cfg);
            PROD_LOG(TAG, "Network open — pairing for %us",
                     (unsigned)cfg.pairing_duration_sec);
            ezb_bdb_open_network(
                (uint8_t)(cfg.pairing_duration_sec > 255
                          ? 255 : cfg.pairing_duration_sec));
            xTaskCreate(pairing_window_task, "pair_win",
                        3072, NULL, 4, NULL);
        } else {
            ESP_LOGW(TAG,
                     "Network formation failed (status=%d) — will retry",
                     (int)status);
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

        PROD_LOG(TAG, "DEVICE_ANNCE short=0x%04hx IEEE=%s",
                 ann->short_addr, ieee_str);
        register_or_update_joined_sensor(ann->short_addr, ieee_str);

        ezb_zdo_active_ep_req_t req = {0};
        req.dst_nwk_addr               = ann->short_addr;
        req.field.nwk_addr_of_interest = ann->short_addr;
        req.cb                         = active_ep_callback;
        req.user_ctx   = (void *)(uintptr_t)ann->short_addr;
        ezb_zdo_active_ep_req(&req);
        break;
    }

    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t duration =
            *(uint8_t *)ezb_app_signal_get_params(app_signal);
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
        ezb_af_endpoint_get_cluster_desc(ep_desc,
                                         EZB_ZCL_CLUSTER_ID_BASIC,
                                         EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc,
        EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)manufacturer_name);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc,
        EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  (void *)model_identifier);

    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc,
        ezb_zcl_on_off_create_cluster_desc(
            NULL, EZB_ZCL_CLUSTER_SERVER)));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc,
        ezb_zcl_occupancy_sensing_create_cluster_desc(
            NULL, EZB_ZCL_CLUSTER_SERVER)));

    /*
     * IAS Zone CLIENT — CRITICAL.
     * Without this the ezbee SDK silently drops all IAS EnrollRequest
     * messages. Door sensors enter an infinite re-join loop.
     */
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc,
        ezb_zcl_ias_zone_create_cluster_desc(
            NULL, EZB_ZCL_CLUSTER_CLIENT)));

    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));

    ezb_zcl_raw_command_handler_register(raw_frame_handler);
    ezb_zcl_core_action_handler_register(
        esp_zigbee_zcl_core_action_handler);

    return ESP_OK;
}

static esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(
        ezb_bdb_set_primary_channel_set(ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(
        ezb_bdb_set_secondary_channel_set(ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(
        ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));
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
                    ESP_LOGW(TAG, "NVS save failed: %s",
                             esp_err_to_name(err));
            }
        }
    }
}

// ============================================================================
// ZIGBEE STACK TASK
// ============================================================================

static void esp_zigbee_stack_main_task(void *pvParameters)
{
    (void)pvParameters;

    esp_zigbee_config_t zigbee_config = {0};
    zigbee_config.device_config.device_type =
        EZB_NWK_DEVICE_TYPE_COORDINATOR;
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
    ESP_ERROR_CHECK(
        nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME));

#if FACTORY_RESET_MODE
    /*
     * FACTORY_RESET_MODE = 1
     * Erases the "nvs" partition only — sensor registry + uart config.
     * The "zb_storage" partition (Zigbee network keys) is preserved.
     * Sensors that are already paired WILL re-join automatically without
     * pressing any button because they still hold valid network credentials.
     * Use this mode when deploying a fresh unit so old test sensor names
     * are wiped. Set FACTORY_RESET_MODE=0 for normal operation.
     */
    ESP_LOGW(TAG, "FACTORY RESET MODE — erasing sensor NVS");
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

    /*
     * UART Master init — after NVS init and load_config so the uart_hub
     * NVS config is available. Before Zigbee stack so UART is ready to
     * receive set_config from Master before sensors start joining.
     */
    esp_err_t uart_err = uart_master_init();
    if (uart_err != ESP_OK) {
        ESP_LOGW(TAG,
                 "UART Master init failed (%s) — running standalone",
                 esp_err_to_name(uart_err));
    }

    /*
     * Notify Master that Sensor Hub has booted.
     * Sent before Zigbee stack starts — Master knows hub is alive
     * even before any sensor joins.
     */
    uart_master_send_hub_boot();

    xTaskCreate(esp_zigbee_stack_main_task, "Zigbee_main",
                4096 * 2, NULL, 5, NULL);
    xTaskCreate(persist_task, "persist_task",
                2048, NULL, 3, NULL);

#if WATCHDOG_ENABLE
    xTaskCreate(sensor_watchdog_task, "watchdog",
                3072, NULL, 2, NULL);
    {
        uart_hub_config_t cfg;
        uart_master_get_config(&cfg);
        PROD_LOG(TAG,
                 "Watchdog enabled — %umin cycle, %us ping timeout",
                 cfg.watchdog_interval_min,
                 cfg.watchdog_ping_timeout_sec);
    }
#else
    PROD_LOG(TAG, "Watchdog disabled (WATCHDOG_ENABLE=0)");
#endif
}