/*
 * main.c — Sensor Hub Zigbee Coordinator
 * Innovatsii EMS — Pico 1
 * Firmware Version: 0.2.5
 *
 * PRODUCTION ARCHITECTURE — V4.2 compliant
 *
 * SENSOR IDENTIFICATION SEQUENCE:
 *   1. DEVICE_ANNCE → register sensor → send active EP request IMMEDIATELY
 *      (no delay — ZG-102Z has only ~400ms awake window)
 *   2. EP response → bind IAS_ZONE + OCCUPANCY_SENSING (2 clusters only)
 *   3. Bind confirmation(s) received → sensor is provably awake → send model ID read
 *   4a. Model ID response → classify type → bind POWER_CONFIG → configure reporting → fading time
 *   4b. Model ID timeout (5s) → infer ZG-102Z (sleepy device that never responds)
 *   4c. First IAS zone data from unidentified sensor → retry model ID read
 *   4d. Temperature report → must be ZG-204ZV; Occupancy report → must be ZG-205Z/A
 *
 * WATCHDOG (V4.2):
 *   DOOR sensors (ZG-102Z): NEVER marked offline. 24-hour silence alert only.
 *   PRESENCE sensors: ping every interval. 2 retries. Immediate alert on failure.
 *
 * BOOT:
 *   Passive — waits for ping from Master.
 *   hub_init → start Zigbee → rejoin → hub_ready → wait for start_watchdog.
 *
 * CRITICAL FLAGS:
 *   FACTORY_RESET_MODE = 0 always. Value 1 erases sensors on every boot.
 *   hub_ready task stack = 3072 (was 2048 — crashed on every boot).
 *   rejoin task stack = 6144 with static arrays (was 3072 — crashed with 3+ sensors).
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
 * FACTORY_RESET_MODE — MUST BE 0 FOR PRODUCTION.
 * 1 erases sensor registry on every boot. Use only for lab deployments.
 */
#define FACTORY_RESET_MODE  0
#define RAW_LOGS_MODE       0
#define WATCHDOG_ENABLE     1

#define ZIGBEE_PRIMARY_CHANNEL_MASK   0x07FFF800UL
#define ZIGBEE_SECONDARY_CHANNEL_MASK 0x00000000UL

#define REJOIN_RETRY_COUNT    6
#define REJOIN_RETRY_DELAY_MS 10000
#define REJOIN_POLL_GAP_MS    500

#define DOOR_PENDING_WINDOW_SEC  30

/*
 * MODEL_ID_TIMEOUT_MS:
 * After bind confirmations arrive we send the model ID read.
 * If no response in this window the sensor is sleepy (ZG-102Z).
 */
#define MODEL_ID_TIMEOUT_MS  5000

/* Default fading time for ZG-204ZV / ZG-205Z/A if config not available */
#define PRESENCE_FADING_TIME_DEFAULT_SEC 30

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

#define PROD_LOG(tag, fmt, ...) do {                             \
    char _ts[10]; uptime_str(_ts, sizeof(_ts));                  \
    ESP_LOGI(tag, "[%s] " fmt, _ts, ##__VA_ARGS__);             \
} while (0)

#if RAW_LOGS_MODE
  #define RAW_LOG(...)           printf(__VA_ARGS__)
  #define DEV_LOG(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#else
  #define RAW_LOG(...)           do {} while (0)
  #define DEV_LOG(tag, fmt, ...) do {} while (0)
#endif

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

extern void uart_cmd_start_pairing(uint16_t duration_sec);
extern void uart_cmd_stop_pairing(void);
extern void uart_cmd_remove_sensor(int idx);
extern void uart_cmd_factory_reset(void);

extern volatile bool g_hub_init_received;
extern volatile bool g_hub_init_mode_debug;

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

/*
 * Initial bind clusters — 2 only, sent after EP response.
 * Minimal ZDO traffic prevents re-announce loop.
 * POWER_CONFIG bound separately after sensor type is confirmed.
 */
static const uint16_t k_initial_bind_clusters[] = {
    CLUSTER_IAS_ZONE,
    CLUSTER_OCCUPANCY_SENSING,
};
#define INITIAL_BIND_COUNT \
    (sizeof(k_initial_bind_clusters) / sizeof(k_initial_bind_clusters[0]))

// ============================================================================
// MODEL DEFINITIONS
// ============================================================================

typedef struct {
    const char   *model_id;
    sensor_type_t type;
    sensor_role_t role;
} sensor_model_def_t;

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

const char *role_str(sensor_role_t r)
{
    switch (r) {
    case ROLE_DOOR:     return "DOOR";
    case ROLE_PRESENCE: return "PRESENCE";
    default:            return "UNKNOWN";
    }
}

// ============================================================================
// GLOBALS
// ============================================================================

hub_config_safe_t            g_config            = {0};
sensor_runtime_meta_t        g_meta[MAX_SENSORS]  = {0};
volatile bool                g_watchdog_started   = false;
volatile int                 g_new_sensor_count   = 0;

static bool pairing_active         = false;
static bool pairing_window_expired = false;
static bool network_formed         = false;
static bool formation_requested    = false;
static bool formation_task_started = false;
static volatile bool g_dirty       = false;
static volatile bool g_rejoin_complete = false;

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

static void restore_meta_from_nvs(hub_config_t *config)
{
    for (int i = 0; i < config->sensor_count; i++) {
        sensor_type_t t = (sensor_type_t)config->sensors[i].sensor_type;
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
        /* Reset all volatile state — sensors rejoin fresh each boot */
        g_meta[i].miss_count            = 0;
        g_meta[i].reporting_configured  = false;
        g_meta[i].fade_sent             = false;
        g_meta[i].bound_once            = false;
        g_meta[i].power_config_bound    = false;
        g_meta[i].enroll_sent           = false;
        g_meta[i].ep_active             = 1;
        g_meta[i].bind_pending          = 0;
        g_meta[i].bind_confirmed        = 0;
        g_meta[i].bind_failed           = 0;
        g_meta[i].model_id_pending      = false;
        g_meta[i].model_id_req_ms       = 0;
        g_meta[i].ping_pending          = false;
    }
}

// ============================================================================
// UNIT OCCUPANCY ENGINE  (call with config lock HELD)
// ============================================================================

static void evaluate_unit_occupancy_locked(hub_config_t *c)
{
    if (!c->door_closed_pending) return;

    if (c->door_closed_at > 0) {
        time_t elapsed = time(NULL) - c->door_closed_at;
        if (elapsed > DOOR_PENDING_WINDOW_SEC) {
            PROD_LOG(TAG, "[UNIT] pending expired after %lds", (long)elapsed);
            c->door_closed_pending = false;
            c->door_closed_at      = 0;
            return;
        }
    }

    bool any_yes      = false;
    bool all_no       = true;
    int  online_count = 0;

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

    /* V4.2: blocked when zero online presence sensors */
    if (online_count == 0) {
        PROD_LOG(TAG, "[UNIT] evaluation blocked — no online presence sensors");
        return;
    }

    unit_occupancy_t new_state = c->unit_state;
    if      (c->unit_state == UNIT_VACANT   && any_yes) new_state = UNIT_OCCUPIED;
    else if (c->unit_state == UNIT_OCCUPIED && all_no)  new_state = UNIT_VACANT;

    if (new_state != c->unit_state) {
        c->door_closed_pending = false;
        c->door_closed_at      = 0;
        c->unit_state          = new_state;
        c->unit_state_changed  = time(NULL);
        mark_dirty();
        PROD_LOG(TAG, "[UNIT] %s | presence=%s",
                 unit_state_str(new_state), snap);
        uart_master_send_unit_occupancy(unit_state_str(new_state));
    } else {
        PROD_LOG(TAG, "[UNIT] %s (no change) | presence=%s",
                 unit_state_str(c->unit_state), snap);
        if (all_no || any_yes) {
            c->door_closed_pending = false;
            c->door_closed_at      = 0;
        }
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
    printf("║  INNOVATSII EMS - SENSOR HUB  v%-6s  Pico 1 Phase 2.5    ║\n",
           FIRMWARE_VERSION);
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
    printf("║ [%s] SENSOR REGISTRY (%d/%d)  Hub: %-8s  Unit: %-8s \n",
           ts, c->sensor_count, MAX_SENSORS,
           c->hub_status.occupied ? "OCCUPIED" : "VACANT",
           unit_state_str(c->unit_state));
    printf("║ Network: %s | Pairing: %s | Watchdog: %s\n",
           network_formed     ? "ACTIVE"   : "FORMING",
           pairing_active     ? "OPEN"     : "CLOSED",
           g_watchdog_started ? "STARTED"  : "WAITING");
    printf("╚══════════════════════════════════════════════════════════════╝\n");

    for (int i = 0; i < c->sensor_count; i++) {
        sensor_t     *s      = &c->sensors[i];
        sensor_type_t t      = (sensor_type_t)s->sensor_type;
        sensor_role_t r      = (sensor_role_t)s->sensor_role;
        const char   *status = s->online ? "[ONLINE] " : "[OFFLINE]";

        printf("  [%d] %-20s %s %-10s %-8s IEEE=%-23s Short=0x%04hx\n",
               i + 1, s->sensor_name, status,
               friendly_name_from_type(t), role_str(r),
               s->ieee_addr, s->short_addr);

        if (t == SENSOR_ZG_204ZV) {
            printf("        presence=%-3s  temp=%.1f°C  hum=%.1f%%"
                   "  batt=%u%%  tamper=%s\n",
                   s->presence ? "YES" : "NO",
                   (double)s->temperature_cdeg / 100.0,
                   (double)s->humidity_cpct    / 100.0,
                   s->battery_pct, s->tamper ? "YES" : "NO");
        } else if (t == SENSOR_ZG_205Z_A) {
            printf("        presence=%-3s  tamper=%s\n",
                   s->presence ? "YES" : "NO",
                   s->tamper   ? "YES" : "NO");
        } else if (t == SENSOR_ZG_102Z || t == SENSOR_ZG_102ZA) {
            printf("        contact=%-6s  batt=%u%%  tamper=%s  batt_low=%s\n",
                   s->contact_open ? "OPEN" : "CLOSED",
                   s->battery_pct,
                   s->tamper      ? "YES" : "NO",
                   s->battery_low ? "YES" : "NO");
        } else {
            printf("        (identifying...)\n");
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
    nvs_set_u8(handle, "pairing_active",  config->pairing_active      ? 1 : 0);
    nvs_set_u8(handle, "pairing_expired", pairing_window_expired       ? 1 : 0);
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
        config->mode                = MODE_PAIRING;
        config->sensor_count        = 0;
        config->pairing_active      = false;
        config->pairing_started     = time(NULL);
        config->unit_state          = UNIT_VACANT;
        config->hub_status.occupied = false;
        config->door_closed_pending = false;
        config->door_closed_at      = 0;
        PROD_LOG(TAG, "NVS empty — waiting for hub_init");
        return ESP_OK;
    }
    if (err != ESP_OK) return err;

    uint8_t mode = MODE_PAIRING;
    nvs_get_u8(handle, "mode", &mode);
    config->mode = (hub_mode_t)mode;

    uint8_t pa = 0;
    nvs_get_u8(handle, "pairing_active", &pa);
    config->pairing_active = (pa != 0);

    uint8_t pe = 0;
    nvs_get_u8(handle, "pairing_expired", &pe);
    pairing_window_expired = (pe != 0);

    uint8_t hub_occ = 0;
    nvs_get_u8(handle, "hub_occupied", &hub_occ);
    config->hub_status.occupied = (hub_occ != 0);

    /* Boot state policy: always start VACANT */
    config->unit_state          = UNIT_VACANT;
    config->door_closed_pending = false;
    config->door_closed_at      = 0;

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

    PROD_LOG(TAG, "Loaded %d sensor(s) from NVS", config->sensor_count);
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

    int  idx    = -1;
    bool is_new = false;

    for (int i = 0; i < c->sensor_count; i++) {
        if (strcmp(c->sensors[i].ieee_addr, ieee) == 0) { idx = i; break; }
    }

    if (idx < 0) {
        if (c->sensor_count >= MAX_SENSORS) {
            unlock_config();
            ESP_LOGW(TAG, "Sensor registry full");
            return;
        }
        idx    = c->sensor_count++;
        is_new = true;
        memset(&c->sensors[idx], 0, sizeof(sensor_t));
        memset(&g_meta[idx],     0, sizeof(sensor_runtime_meta_t));
    } else {
        /*
         * Known sensor re-announced — update short address and reset
         * volatile identification state so the full sequence runs fresh.
         *
         * bound_once is NOT reset here. It tracks that IAS+OCC binds were
         * done once. Resetting it causes bind flooding on re-announce.
         * bound_once is only reset in uart_cmd_remove_sensor().
         *
         * power_config_bound IS reset so battery binding re-runs with
         * the new short address.
         *
         * presence cleared — ZG-204ZV stuck-YES fix (V4.2 §13.6).
         */
        g_meta[idx].reporting_configured = false;
        g_meta[idx].fade_sent            = false;
        g_meta[idx].power_config_bound   = false;
        g_meta[idx].enroll_sent          = false;
        g_meta[idx].ping_pending         = false;
        g_meta[idx].miss_count           = 0;
        g_meta[idx].bind_pending         = 0;
        g_meta[idx].bind_confirmed       = 0;
        g_meta[idx].bind_failed          = 0;
        g_meta[idx].model_id_pending     = false;
        g_meta[idx].model_id_req_ms      = 0;
        c->sensors[idx].presence         = false;
    }

    c->sensors[idx].short_addr = short_addr;
    strncpy(c->sensors[idx].ieee_addr, ieee, IEEE_ADDR_STR_LEN - 1);
    c->sensors[idx].ieee_addr[IEEE_ADDR_STR_LEN - 1] = '\0';
    c->sensors[idx].endpoint   = 1;
    c->sensors[idx].last_seen  = time(NULL);
    c->sensors[idx].online     = true;
    set_default_sensor_name(&c->sensors[idx], idx);

    if (is_new && pairing_active) {
        sensor_t *s = &c->sensors[idx];
        unlock_config();
        uart_master_send_new_sensor_joined(
            idx,
            s->sensor_name,
            friendly_name_from_type((sensor_type_t)s->sensor_type),
            role_str((sensor_role_t)s->sensor_role));
        g_new_sensor_count++;
        mark_dirty();
        return;
    }

    unlock_config();
    mark_dirty();
}

// ============================================================================
// FADING TIME — deferred task at file scope
// C does not allow nested functions — must be at file scope.
// ============================================================================

typedef struct { uint16_t sa; uint8_t ep; uint32_t fs; } fade_args_t;

static void fade_task(void *arg)
{
    fade_args_t *x = (fade_args_t *)arg;

    vTaskDelay(pdMS_TO_TICKS(2000));

    uint8_t payload[] = {
        0x00, TUYA_DP_FADING_TIME, 0x02, 0x00, 0x04,
        (uint8_t)((x->fs >> 24) & 0xFF),
        (uint8_t)((x->fs >> 16) & 0xFF),
        (uint8_t)((x->fs >>  8) & 0xFF),
        (uint8_t)((x->fs      ) & 0xFF)
    };

    ezb_zcl_custom_cluster_cmd_t cmd = {0};
    cmd.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
    cmd.cmd_ctrl.dst_addr.u.short_addr = x->sa;
    cmd.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
    cmd.cmd_ctrl.dst_ep                = x->ep;
    cmd.cmd_ctrl.cluster_id            = CLUSTER_PRIVATE_TUYA;
    cmd.cmd_ctrl.fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd.cmd_ctrl.fc.dis_default_rsp    = 1;
    cmd.cmd_id                         = 0x00;
    cmd.data_length                    = sizeof(payload);
    cmd.data                           = payload;

    (void)ezb_zcl_custom_cluster_cmd_req(&cmd);

    PROD_LOG("SENSOR_HUB", "[CFG] fading_time=%lus → 0x%04hx ep%u",
             (unsigned long)x->fs, x->sa, x->ep);

    free(x);
    vTaskDelete(NULL);
}

// ============================================================================
// APPLY MODEL TO SENSOR
// Sets sensor_type, sensor_role, model_id from a confirmed modelID string.
// Returns true if model was recognised.
// ============================================================================

static bool apply_model_to_sensor(int idx, const char *model_id)
{
    const sensor_model_def_t *def = find_model_def(model_id);
    if (!def) {
        ESP_LOGW(TAG, "Sensor %d: unknown modelID '%s'", idx + 1, model_id);
        return false;
    }
    strncpy(g_meta[idx].model_id, model_id,
            sizeof(g_meta[idx].model_id) - 1);
    g_meta[idx].model_id[sizeof(g_meta[idx].model_id) - 1] = '\0';
    g_meta[idx].model_known = true;

    hub_config_t *c = lock_config();
    if (c) {
        c->sensors[idx].sensor_type = (uint8_t)def->type;
        c->sensors[idx].sensor_role = (uint8_t)def->role;
        PROD_LOG(TAG, "[ID] Sensor_%d | %-10s | IEEE=%s | Short=0x%04hx | Role=%s",
                 idx + 1,
                 friendly_name_from_type(def->type),
                 c->sensors[idx].ieee_addr,
                 c->sensors[idx].short_addr,
                 role_str(def->role));
        unlock_config();
    }
    mark_dirty();
    return true;
}

// ============================================================================
// INFER AS DOOR SENSOR (ZG-102Z fallback)
// Called when model ID times out but sensor has sent IAS zone status.
// ============================================================================

static void infer_as_door_sensor(int idx)
{
    hub_config_t *c = lock_config();
    if (!c) return;

    if ((sensor_type_t)c->sensors[idx].sensor_type != SENSOR_UNKNOWN) {
        unlock_config();
        return;
    }

    c->sensors[idx].sensor_type = (uint8_t)SENSOR_ZG_102Z;
    c->sensors[idx].sensor_role = (uint8_t)ROLE_DOOR;
    strncpy(g_meta[idx].model_id, "ZG-102Z",
            sizeof(g_meta[idx].model_id) - 1);
    g_meta[idx].model_known = true;

    PROD_LOG(TAG, "[ID] Sensor_%d inferred as ZG-102Z (DOOR) — "
             "model ID timed out (sleepy device)", idx + 1);
    unlock_config();
    mark_dirty();
}

// ============================================================================
// POST-IDENTIFICATION SETUP
// Called after sensor type is confirmed.
// Binds POWER_CONFIG, sends configure reporting, sends fading time.
// Called from Zigbee stack context OR timeout task — uses Zigbee API directly.
// ============================================================================

static void post_identification_setup(int idx, uint16_t short_addr, uint8_t ep)
{
    if (idx < 0) return;

    hub_config_t *c = lock_config();
    if (!c) return;
    sensor_type_t type = (sensor_type_t)c->sensors[idx].sensor_type;
    char name[SENSOR_NAME_LEN];
    strncpy(name, c->sensors[idx].sensor_name, SENSOR_NAME_LEN - 1);
    name[SENSOR_NAME_LEN - 1] = '\0';
    unlock_config();

    if (type == SENSOR_UNKNOWN) return;

    /* ── Bind POWER_CONFIG ── */
    if (!g_meta[idx].power_config_bound) {
        ezb_extaddr_t sensor_ieee;
        ezb_extaddr_t coordinator_ieee;
        ezb_get_extended_address(&coordinator_ieee);

        if (ezb_address_extended_by_short(short_addr, &sensor_ieee)
                == EZB_ERR_NONE) {
            ezb_zdo_bind_req_t req = {0};
            req.dst_nwk_addr                 = short_addr;
            req.field.src_addr               = sensor_ieee;
            req.field.src_ep                 = ep;
            req.field.cluster_id             = CLUSTER_POWER_CONFIG;
            req.field.dst_addr_mode          = EZB_ADDR_MODE_EXT;
            req.field.dst_addr.extended_addr = coordinator_ieee;
            req.field.dst_ep                 = COORDINATOR_ENDPOINT;
            req.cb                           = NULL;
            req.user_ctx                     = NULL;
            ezb_zdo_bind_req(&req);
            g_meta[idx].power_config_bound = true;
            PROD_LOG(TAG, "[BIND] POWER_CONFIG → %s 0x%04hx", name, short_addr);
        }
    }

    /* ── Configure reporting ── */
    if (!g_meta[idx].reporting_configured) {
        ezb_zcl_config_report_cmd_t cmd = {0};
        cmd.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
        cmd.cmd_ctrl.dst_addr.u.short_addr = short_addr;
        cmd.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
        cmd.cmd_ctrl.dst_ep                = ep;
        cmd.cmd_ctrl.fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV;
        cmd.cmd_ctrl.fc.dis_default_rsp    = 1;
        cmd.payload.record_number          = 1;

        if (type == SENSOR_ZG_204ZV) {
            ezb_zcl_config_report_record_t ias = {
                .direction = EZB_ZCL_REPORTING_SEND,
                .attr_id   = ATTR_IAS_ZONE_STATUS,
                .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT16,
                              .min_interval = 2, .max_interval = 120,
                              .reportable_change = {.u16 = 1}},
            };
            cmd.cmd_ctrl.cluster_id  = CLUSTER_IAS_ZONE;
            cmd.payload.record_field = &ias;
            (void)ezb_zcl_config_report_cmd_req(&cmd);

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

            ezb_zcl_config_report_record_t batt_z = {
                .direction = EZB_ZCL_REPORTING_SEND,
                .attr_id   = ATTR_BATTERY_PERCENT,
                .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT8,
                              .min_interval = 60, .max_interval = 3600,
                              .reportable_change = {.u8 = 2}},
            };
            cmd.cmd_ctrl.cluster_id  = CLUSTER_POWER_CONFIG;
            cmd.payload.record_field = &batt_z;
            (void)ezb_zcl_config_report_cmd_req(&cmd);

        } else if (type == SENSOR_ZG_205Z_A) {
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

            ezb_zcl_config_report_record_t batt_a = {
                .direction = EZB_ZCL_REPORTING_SEND,
                .attr_id   = ATTR_BATTERY_PERCENT,
                .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT8,
                              .min_interval = 60, .max_interval = 3600,
                              .reportable_change = {.u8 = 2}},
            };
            cmd.cmd_ctrl.cluster_id  = CLUSTER_POWER_CONFIG;
            cmd.payload.record_field = &batt_a;
            (void)ezb_zcl_config_report_cmd_req(&cmd);

        } else if (type == SENSOR_ZG_102Z || type == SENSOR_ZG_102ZA) {
            ezb_zcl_config_report_record_t batt_d = {
                .direction = EZB_ZCL_REPORTING_SEND,
                .attr_id   = ATTR_BATTERY_PERCENT,
                .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT8,
                              .min_interval = 60, .max_interval = 3600,
                              .reportable_change = {.u8 = 2}},
            };
            cmd.cmd_ctrl.cluster_id  = CLUSTER_POWER_CONFIG;
            cmd.payload.record_field = &batt_d;
            (void)ezb_zcl_config_report_cmd_req(&cmd);
        }

        g_meta[idx].reporting_configured = true;
        PROD_LOG(TAG, "[CFG] reporting configured — %s (%s)",
                 name, friendly_name_from_type(type));
    }

    /* ── Fading time for presence sensors ── */
    if ((type == SENSOR_ZG_204ZV || type == SENSOR_ZG_205Z_A)
            && !g_meta[idx].fade_sent) {
        uart_hub_config_t cfg;
        uart_master_get_config(&cfg);
        uint32_t fade_sec = cfg.presence_fading_time_sec;
        if (fade_sec < 5 || fade_sec > 300)
            fade_sec = PRESENCE_FADING_TIME_DEFAULT_SEC;

        fade_args_t *fa = malloc(sizeof(fade_args_t));
        if (fa) {
            fa->sa = short_addr;
            fa->ep = ep;
            fa->fs = fade_sec;
            xTaskCreate(fade_task, "fade", 2048, fa, 3, NULL);
        }
        g_meta[idx].fade_sent = true;
    }

    PROD_LOG(TAG, "[ID] %s OPERATIONAL | type=%s",
             name, friendly_name_from_type(type));
}

// ============================================================================
// MODEL ID REQUEST
// Sent after bind confirmations — sensor is proven awake at this point.
// ============================================================================

static void request_model_id(int idx, uint16_t short_addr, uint8_t ep)
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
    (void)ezb_zcl_read_attr_cmd_req(&cmd);

    g_meta[idx].model_id_pending = true;
    g_meta[idx].model_id_req_ms  = (uint32_t)(esp_timer_get_time() / 1000ULL);

    PROD_LOG(TAG, "[ID] model ID read sent → 0x%04hx ep%u", short_addr, ep);
}

// ============================================================================
// WATCHDOG PING
// External task context — needs Zigbee lock.
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
// BIND CALLBACK
//
// user_ctx encodes: (sensor_idx << 16) | short_addr
// When all pending binds have been answered (confirmed + failed == pending),
// send the model ID read. At this point the sensor has proven it is awake.
// ============================================================================

static void bind_result_callback(const ezb_zdp_bind_req_result_t *result,
                                  void *user_ctx)
{
    uintptr_t ctx        = (uintptr_t)user_ctx;
    int       idx        = (int)((ctx >> 16) & 0xFFFF);
    uint16_t  short_addr = (uint16_t)(ctx & 0xFFFF);

    if (idx < 0 || idx >= MAX_SENSORS) return;

    bool ok = (result && result->error == EZB_ERR_NONE
               && result->rsp && result->rsp->status == 0x00);

    if (ok) {
        g_meta[idx].bind_confirmed++;
        PROD_LOG(TAG, "[BIND] ok → Sensor_%d 0x%04hx", idx + 1, short_addr);
    } else {
        g_meta[idx].bind_failed++;
        PROD_LOG(TAG, "[BIND] fail → Sensor_%d 0x%04hx err=0x%04x",
                 idx + 1, short_addr, result ? result->error : 0xFFFF);
    }

    uint8_t total_answered = g_meta[idx].bind_confirmed + g_meta[idx].bind_failed;
    if (total_answered < g_meta[idx].bind_pending) return;

    /* All bind requests answered — now send model ID read */
    g_meta[idx].bound_once = true;

    uint8_t ep = g_meta[idx].ep_active;
    if (ep == 0) ep = 1;

    if (g_meta[idx].bind_confirmed > 0 && !g_meta[idx].model_id_pending) {
        /*
         * At least one bind confirmed — sensor is awake and responsive.
         * Send model ID read. ZG-204ZV and ZG-205Z/A respond immediately.
         * ZG-102Z may or may not respond (may have gone back to sleep).
         */
        request_model_id(idx, short_addr, ep);
    } else if (g_meta[idx].bind_confirmed == 0) {
        /*
         * All binds timed out — sensor went back to sleep (ZG-102Z).
         * model_id_timeout_task will classify it after MODEL_ID_TIMEOUT_MS.
         * Set model_id_req_ms now so the timeout counts from bind completion.
         */
        g_meta[idx].model_id_pending = true;
        g_meta[idx].model_id_req_ms  = (uint32_t)(esp_timer_get_time() / 1000ULL);
        PROD_LOG(TAG, "[ID] Sensor_%d all binds timed out — "
                 "will infer as ZG-102Z after %dms",
                 idx + 1, MODEL_ID_TIMEOUT_MS);
    }
}

// ============================================================================
// ACTIVE EP CALLBACK
// Fires immediately after DEVICE_ANNCE (no delay).
// Sends 2 bind requests only — minimal ZDO traffic prevents re-announce.
// ============================================================================

static void active_ep_callback(
        const ezb_zdo_active_ep_req_result_t *result, void *user_ctx)
{
    uint16_t short_addr = (uint16_t)(uintptr_t)user_ctx;
    if (!result || result->error != EZB_ERR_NONE || !result->rsp) {
        ESP_LOGW(TAG, "[EP] query failed for 0x%04hx err=%d",
                 short_addr, result ? result->error : -1);
        return;
    }

    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0) return;

    if (result->rsp->active_ep_count == 0) {
        ESP_LOGW(TAG, "[EP] 0 endpoints for 0x%04hx", short_addr);
        return;
    }

    uint8_t ep = result->rsp->active_ep_list[0];
    g_meta[idx].ep_active = ep;

    PROD_LOG(TAG, "[EP] 0x%04hx has %u endpoint(s) — binding %zu clusters",
             short_addr, result->rsp->active_ep_count, INITIAL_BIND_COUNT);

    ezb_extaddr_t sensor_ieee;
    if (ezb_address_extended_by_short(short_addr, &sensor_ieee) != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "[EP] IEEE resolve failed for 0x%04hx", short_addr);
        return;
    }

    ezb_extaddr_t coordinator_ieee;
    ezb_get_extended_address(&coordinator_ieee);

    g_meta[idx].bind_pending   = (uint8_t)INITIAL_BIND_COUNT;
    g_meta[idx].bind_confirmed = 0;
    g_meta[idx].bind_failed    = 0;

    for (size_t i = 0; i < INITIAL_BIND_COUNT; i++) {
        ezb_zdo_bind_req_t req = {0};
        req.dst_nwk_addr                 = short_addr;
        req.field.src_addr               = sensor_ieee;
        req.field.src_ep                 = ep;
        req.field.cluster_id             = k_initial_bind_clusters[i];
        req.field.dst_addr_mode          = EZB_ADDR_MODE_EXT;
        req.field.dst_addr.extended_addr = coordinator_ieee;
        req.field.dst_ep                 = COORDINATOR_ENDPOINT;
        req.cb                           = bind_result_callback;
        /* encode idx and short_addr into user_ctx */
        req.user_ctx = (void *)(uintptr_t)(
            ((uint32_t)(uint16_t)idx << 16) | (uint32_t)short_addr);
        ezb_zdo_bind_req(&req);
    }
}

// ============================================================================
// MODEL ID TIMEOUT CHECKER TASK
// Runs every second. Checks if any sensor's model ID read timed out.
// If yes and sensor is still UNKNOWN, infers as ZG-102Z (sleepy door sensor).
// ============================================================================

static void model_id_timeout_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        hub_config_t *c = lock_config();
        if (!c) continue;
        int count = c->sensor_count;
        unlock_config();

        for (int i = 0; i < count; i++) {
            if (!g_meta[i].model_id_pending) continue;

            uint32_t elapsed = now_ms - g_meta[i].model_id_req_ms;
            if (elapsed < MODEL_ID_TIMEOUT_MS) continue;

            /* Model ID timed out */
            g_meta[i].model_id_pending = false;

            hub_config_t *c2 = lock_config();
            if (!c2) continue;
            uint16_t short_addr = c2->sensors[i].short_addr;
            uint8_t  ep         = g_meta[i].ep_active;
            if (ep == 0) ep = 1;
            sensor_type_t tp = (sensor_type_t)c2->sensors[i].sensor_type;
            unlock_config();

            if (tp == SENSOR_UNKNOWN) {
                PROD_LOG(TAG,
                         "[ID] Sensor_%d model ID timed out — "
                         "inferring ZG-102Z (sleepy device)", i + 1);
                infer_as_door_sensor(i);
                post_identification_setup(i, short_addr, ep);
            }
        }
    }
}

// ============================================================================
// ZCL READ ATTR RESPONSE
// Handles model ID responses (from request_model_id) and watchdog pings.
// ============================================================================

static void zcl_core_read_attr_rsp_handler(
        ezb_zcl_cmd_read_attr_rsp_message_t *message)
{
    if (!message || !message->in.header) return;
    if (message->info.cluster_id != CLUSTER_BASIC) return;

    uint16_t short_addr = message->in.header->src_addr.u.short_addr;
    uint8_t  src_ep     = message->in.header->src_ep;
    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0) return;

    /* Mark sensor online and clear watchdog state */
    {
        hub_config_t *c = lock_config();
        if (c) {
            bool was_offline = !c->sensors[idx].online;
            c->sensors[idx].online    = true;
            c->sensors[idx].last_seen = time(NULL);
            g_meta[idx].ping_pending  = false;
            g_meta[idx].miss_count    = 0;
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

    /* Process model ID if a read was pending */
    if (!g_meta[idx].model_id_pending) return;

    g_meta[idx].model_id_pending = false;

    bool model_applied = false;
    ezb_zcl_read_attr_rsp_variable_t *var = message->in.variables;
    while (var) {
        if (var->status == EZB_ZCL_STATUS_SUCCESS &&
                var->attr_id == ATTR_BASIC_MODEL_IDENTIFIER) {
            uint8_t len = *(uint8_t *)var->attr_value;
            char model[32] = {0};
            if (len >= sizeof(model)) len = (uint8_t)(sizeof(model) - 1);
            memcpy(model, (char *)(var->attr_value + 1), len);
            model[len] = '\0';
            model_applied = apply_model_to_sensor(idx, model);
        }
        var = var->next;
    }

    if (model_applied) {
        uint8_t ep = g_meta[idx].ep_active;
        if (ep == 0) ep = src_ep;
        if (ep == 0) ep = 1;
        post_identification_setup(idx, short_addr, ep);
    }
}

// ============================================================================
// IAS ZONE ENROLL HANDLER
// ============================================================================

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
    PROD_LOG(TAG, "[IAS] EnrollRsp → 0x%04hx zone_id=%u ret=0x%04x",
             short_addr, zone_id, ret);

    if (idx >= 0) {
        hub_config_t *c = lock_config();
        if (c) {
            c->sensors[idx].online    = true;
            c->sensors[idx].last_seen = time(NULL);
            g_meta[idx].enroll_sent   = true;
            g_meta[idx].ping_pending  = false;
            g_meta[idx].miss_count    = 0;
            unlock_config();
        }
    }
}

// ============================================================================
// IAS ZONE STATUS CHANGE HANDLER
//
// ZG-102Z fallback identification:
// If sensor type is UNKNOWN and IAS zone status arrives, the sensor woke up
// (door event) before or while model ID read was pending.
// The sensor is now awake — retry model ID read immediately.
// If model ID still times out, model_id_timeout_task will infer ZG-102Z.
// ============================================================================

static void zcl_ias_zone_status_change_handler(
        ezb_zcl_ias_zone_status_change_notif_message_t *message)
{
    if (!message || !message->in.header) return;

    uint16_t short_addr  = message->in.header->src_addr.u.short_addr;
    uint8_t  src_ep      = message->in.header->src_ep;
    uint16_t zone_status = message->in.payload.zone_status;
    bool alarm1   = (zone_status & 0x0001) != 0;
    bool tamper   = (zone_status & 0x0004) != 0;
    bool batt_low = (zone_status & 0x0008) != 0;

    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0) {
        ESP_LOGW(TAG, "IAS from unknown 0x%04hx zone=0x%04hx — "
                 "sensor not in registry (may be old paired sensor)",
                 short_addr, zone_status);
        return;
    }

    hub_config_t *c = lock_config();
    if (!c) return;

    bool was_offline = !c->sensors[idx].online;
    c->sensors[idx].online    = true;
    c->sensors[idx].last_seen = time(NULL);
    g_meta[idx].ping_pending  = false;
    g_meta[idx].miss_count    = 0;

    if (was_offline) {
        PROD_LOG(TAG, "[WDG] %s back ONLINE", c->sensors[idx].sensor_name);
        uart_master_send_sensor_health(c->sensors[idx].sensor_name, true);
    }

    sensor_type_t t    = (sensor_type_t)c->sensors[idx].sensor_type;
    sensor_role_t role = (sensor_role_t)c->sensors[idx].sensor_role;
    sensor_t     *s    = &c->sensors[idx];

    s->tamper      = tamper;
    s->battery_low = batt_low;

    /*
     * ZG-102Z fallback: sensor is UNKNOWN but just sent IAS zone status.
     * It is awake right now — send model ID read immediately.
     * model_id_timeout_task will infer ZG-102Z if read times out.
     */
    if (t == SENSOR_UNKNOWN) {
        uint8_t ep = g_meta[idx].ep_active;
        if (ep == 0) ep = src_ep;
        if (ep == 0) ep = 1;

        /* Store contact state best-guess while we wait for identification */
        s->contact_open = alarm1;
        unlock_config();

        if (!g_meta[idx].model_id_pending) {
            PROD_LOG(TAG, "[ID] Sensor_%d: IAS from unidentified sensor — "
                     "retrying model ID read (sensor is awake)",
                     idx + 1);
            request_model_id(idx, short_addr, ep);
        }
        /* Data will be processed correctly once type is identified */
        return;
    }

    /* Process IAS data for identified sensors */
    if (role == ROLE_DOOR) {
        bool state_changed = (s->contact_open != alarm1);
        bool was_open      = s->contact_open;
        s->contact_open    = alarm1;

        if (state_changed) {
            s->last_change = time(NULL);
            PROD_LOG(TAG, "[DATA] %s [%s] contact=%s",
                     s->sensor_name, friendly_name_from_type(t),
                     alarm1 ? "OPEN" : "CLOSED");
            uart_master_send_door(s->sensor_name, alarm1);
            uart_master_notify_door_state(idx, s->sensor_name, alarm1);

            if (was_open && !alarm1) {
                c->door_closed_pending = true;
                c->door_closed_at      = time(NULL);
                PROD_LOG(TAG, "[UNIT] door closed — pending %ds",
                         DOOR_PENDING_WINDOW_SEC);
                evaluate_unit_occupancy_locked(c);
            }
        }

    } else if (role == ROLE_PRESENCE ||
               t    == SENSOR_ZG_204ZV  ||
               t    == SENSOR_ZG_205Z_A) {

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
        s->presence = alarm1;
        if (alarm1) update_hub_presence_locked(c);
        PROD_LOG(TAG, "[DATA] Sensor_%d unclassified IAS alarm1=%d",
                 idx + 1, alarm1);
    }

    unlock_config();
    mark_dirty();
}

// ============================================================================
// REPORT ATTRIBUTE HANDLER
//
// Also used as secondary identification:
//   Temperature report (0x0402) → must be ZG-204ZV
//   Occupancy report  (0x0406) → must be ZG-205Z/A
// ============================================================================

static void zcl_core_cmd_report_attr_handler(
        ezb_zcl_cmd_report_attr_message_t *message)
{
    if (!message || !message->in.header || !message->in.variables) return;

    uint16_t short_addr = message->in.header->src_addr.u.short_addr;
    uint8_t  src_ep     = message->in.header->src_ep;
    uint16_t cluster_id = message->info.cluster_id;

    if (cluster_id == CLUSTER_ILLUMINANCE) return;

    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0) return;

    hub_config_t *c = lock_config();
    if (!c) return;

    bool was_offline = !c->sensors[idx].online;
    c->sensors[idx].online    = true;
    c->sensors[idx].last_seen = time(NULL);
    g_meta[idx].ping_pending  = false;
    g_meta[idx].miss_count    = 0;
    if (was_offline) {
        PROD_LOG(TAG, "[WDG] %s back ONLINE", c->sensors[idx].sensor_name);
        uart_master_send_sensor_health(c->sensors[idx].sensor_name, true);
        update_hub_presence_locked(c);
    }

    sensor_t     *s       = &c->sensors[idx];
    sensor_type_t tp      = (sensor_type_t)s->sensor_type;
    bool          changed = false;

    /*
     * Secondary identification from attribute report cluster.
     * Only temperature and occupancy clusters are definitive:
     *   Temperature (0x0402) → ZG-204ZV (only mmWave with temp sensor)
     *   Occupancy   (0x0406) → ZG-205Z/A
     */
    if (tp == SENSOR_UNKNOWN) {
        uint8_t ep = g_meta[idx].ep_active;
        if (ep == 0) ep = src_ep;
        if (ep == 0) ep = 1;

        const char *inferred_model = NULL;
        if (cluster_id == CLUSTER_TEMP_MEASUREMENT ||
            cluster_id == CLUSTER_HUMIDITY) {
            inferred_model = "ZG-204ZV";
        } else if (cluster_id == CLUSTER_OCCUPANCY_SENSING) {
            inferred_model = "CK-BL702-MWS-01(7016)";
        }

        if (inferred_model) {
            PROD_LOG(TAG, "[ID] Sensor_%d: cluster 0x%04hx → infer %s",
                     idx + 1, cluster_id, inferred_model);
            unlock_config();
            if (apply_model_to_sensor(idx, inferred_model)) {
                post_identification_setup(idx, short_addr, ep);
            }
            c = lock_config();
            if (!c) return;
            s  = &c->sensors[idx];
            tp = (sensor_type_t)s->sensor_type;
        } else {
            unlock_config();
            return;
        }
    }

    /* Process data per confirmed sensor type */
    if (tp == SENSOR_ZG_204ZV) {
        if (cluster_id == CLUSTER_TEMP_MEASUREMENT) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables;
                     v; v = v->next) {
                if (v->attr_id == ATTR_TEMPERATURE_MEASURED) {
                    s->temperature_cdeg = *(int16_t *)v->attr_value;
                    changed = true;
                    PROD_LOG(TAG, "[DATA] %s temp=%.2f°C", s->sensor_name,
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
                    PROD_LOG(TAG, "[DATA] %s hum=%.2f%%", s->sensor_name,
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
    } else if (tp == SENSOR_ZG_205Z_A) {
        if (cluster_id == CLUSTER_OCCUPANCY_SENSING) {
            for (ezb_zcl_report_attr_variable_t *v = message->in.variables;
                     v; v = v->next) {
                if (v->attr_id == ATTR_OCCUPANCY) {
                    bool occ        = (*(uint8_t *)v->attr_value) != 0;
                    bool state_chgd = (s->presence != occ);
                    s->presence = occ;
                    changed     = true;
                    if (state_chgd) {
                        s->last_change = time(NULL);
                        PROD_LOG(TAG, "[DATA] %s [ZG-205Z/A] presence=%s",
                                 s->sensor_name, occ ? "YES" : "NO");
                        update_hub_presence_locked(c);
                        uart_master_send_sensor_presence(
                            s->sensor_name,
                            friendly_name_from_type(tp), occ);
                    }
                    if (c->door_closed_pending)
                        evaluate_unit_occupancy_locked(c);
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
    } else if (tp == SENSOR_ZG_102Z || tp == SENSOR_ZG_102ZA) {
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
        RAW_LOG("[RAW] ZCL 0x%04lx (unhandled)\n",
                (unsigned long)callback_id);
        break;
    }
}

static bool raw_frame_handler(const ezb_zcl_raw_frame_t *raw_frame)
{
    if (!raw_frame || !raw_frame->header) return false;
    uint16_t cluster_id = raw_frame->header->cluster_id;
    if (cluster_id == CLUSTER_PRIVATE_TUYA && raw_frame->payload_length > 0) {
        RAW_LOG("[RAW] EF00 src=0x%04hx:",
                raw_frame->header->src_addr.u.short_addr);
        for (uint16_t i = 0; i < raw_frame->payload_length; i++)
            RAW_LOG(" %02X", raw_frame->payload[i]);
        RAW_LOG("\n");
    }
    return false;
}

// ============================================================================
// WATCHDOG TASK
// V4.2: Door sensors NEVER marked offline. 24-hour silence alert only.
//       Presence sensors: ping, 2 retries, immediate offline alert.
// ============================================================================

#if WATCHDOG_ENABLE
static void sensor_watchdog_task(void *arg)
{
    (void)arg;

    uart_hub_config_t cfg;
    uart_master_get_config(&cfg);

    while (!g_watchdog_started) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    PROD_LOG(TAG, "[WDG] Watchdog started — %umin cycle, %us ping timeout",
             cfg.watchdog_interval_min, cfg.watchdog_ping_timeout_sec);

    vTaskDelay(pdMS_TO_TICKS(
        (uint32_t)cfg.watchdog_interval_min * 60UL * 1000UL));

    for (;;) {
        uart_master_get_config(&cfg);
        uint32_t interval_ms = (uint32_t)cfg.watchdog_interval_min * 60UL * 1000UL;
        uint32_t ping_ms     = (uint32_t)cfg.watchdog_ping_timeout_sec * 1000UL;

        hub_config_t *cc = lock_config();
        int count = cc ? cc->sensor_count : 0;
        if (cc) unlock_config();

        PROD_LOG(TAG, "[WDG] Health check (%d sensors)", count);

        for (int i = 0; i < count; i++) {
            hub_config_t *c = lock_config();
            if (!c) continue;

            bool          online     = c->sensors[i].online;
            uint16_t      short_addr = c->sensors[i].short_addr;
            uint8_t       ep         = c->sensors[i].endpoint;
            sensor_role_t role       = (sensor_role_t)c->sensors[i].sensor_role;
            time_t        last_seen  = c->sensors[i].last_seen;
            char          name[SENSOR_NAME_LEN];
            strncpy(name, c->sensors[i].sensor_name, SENSOR_NAME_LEN - 1);
            name[SENSOR_NAME_LEN - 1] = '\0';
            unlock_config();

            /*
             * DOOR sensors — NEVER mark offline (V4.2 §16.6).
             * ZG-102Z is sleepy. Radio off between events is normal.
             * Only alert if silent for door_sensor_max_silence_hours.
             */
            if (role == ROLE_DOOR) {
                if (last_seen > 0 && cfg.door_sensor_max_silence_hours > 0) {
                    time_t silence   = time(NULL) - last_seen;
                    time_t threshold = (time_t)cfg.door_sensor_max_silence_hours
                                       * 3600;
                    if (silence > threshold) {
                        PROD_LOG(TAG,
                                 "[WDG] %s door silent %ldh (threshold=%uh) — alert",
                                 name, (long)(silence / 3600),
                                 cfg.door_sensor_max_silence_hours);
                        uart_master_send_sensor_health(name, false);
                        /* Reset last_seen so alert fires once per threshold */
                        hub_config_t *cu = lock_config();
                        if (cu) {
                            cu->sensors[i].last_seen = time(NULL);
                            unlock_config();
                        }
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            /* PRESENCE sensors — standard ZCL ping */
            if (!online) {
                g_meta[i].miss_count++;
                if (g_meta[i].miss_count >= WATCHDOG_OFFLINE_PAIRING_THRESHOLD) {
                    PROD_LOG(TAG, "[WDG] %s offline %u cycles — opening pairing",
                             name, g_meta[i].miss_count);
                    g_meta[i].miss_count = 0;
                    uart_cmd_start_pairing(WATCHDOG_PAIRING_REOPEN_SEC);
                }
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            g_meta[i].ping_pending = true;
            send_ping(short_addr, ep);
            vTaskDelay(pdMS_TO_TICKS(ping_ms));

            if (g_meta[i].ping_pending) {
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
                g_meta[i].miss_count   = 1;
            } else {
                PROD_LOG(TAG, "[WDG] %s ONLINE", name);
                g_meta[i].miss_count = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(500));
        }

        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
}
#endif

// ============================================================================
// REJOIN TASK
// Static local arrays prevent stack overflow with 3+ sensors.
// rejoin_task runs exactly once per boot — static is safe.
// ============================================================================

static void rejoin_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(2000));

    hub_config_t *c = lock_config();
    if (!c || c->sensor_count == 0) {
        if (c) unlock_config();
        PROD_LOG(TAG, "[JOIN] No sensors in NVS — rejoin skipped");
        g_rejoin_complete = true;
        vTaskDelete(NULL);
        return;
    }

    int count = c->sensor_count;
    PROD_LOG(TAG, "[JOIN] Rejoining %d sensor(s)", count);

    /*
     * Static arrays: sscanf() uses ~700 bytes on ESP32-C6.
     * With dynamic arrays + sscanf + ZDO calls the 3072-byte stack
     * was exhausted at attempt 6 with 3 sensors — confirmed crash.
     */
    static char          ieee_list[MAX_SENSORS][IEEE_ADDR_STR_LEN];
    static uint16_t      short_list[MAX_SENSORS];
    static sensor_type_t type_list[MAX_SENSORS];
    static char          name_list[MAX_SENSORS][SENSOR_NAME_LEN];
    static uint8_t       role_list[MAX_SENSORS];

    for (int i = 0; i < count; i++) {
        strncpy(ieee_list[i],  c->sensors[i].ieee_addr,   IEEE_ADDR_STR_LEN - 1);
        strncpy(name_list[i],  c->sensors[i].sensor_name, SENSOR_NAME_LEN   - 1);
        ieee_list[i][IEEE_ADDR_STR_LEN - 1] = '\0';
        name_list[i][SENSOR_NAME_LEN   - 1] = '\0';
        short_list[i] = c->sensors[i].short_addr;
        type_list[i]  = (sensor_type_t)c->sensors[i].sensor_type;
        role_list[i]  = c->sensors[i].sensor_role;
    }
    unlock_config();

    int total_online = 0, total_offline = 0;

    for (int i = 0; i < count; i++) {
        if (ieee_list[i][0] == '\0') {
            total_offline++;
            uart_master_send_sensor_status(i, name_list[i],
                friendly_name_from_type(type_list[i]),
                role_str((sensor_role_t)role_list[i]), false);
            continue;
        }

        ezb_extaddr_t target_ieee = {0};
        unsigned int  bytes[8]    = {0};
        int parsed = sscanf(ieee_list[i],
                            "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                            &bytes[7], &bytes[6], &bytes[5], &bytes[4],
                            &bytes[3], &bytes[2], &bytes[1], &bytes[0]);
        if (parsed != 8) {
            total_offline++;
            uart_master_send_sensor_status(i, name_list[i],
                friendly_name_from_type(type_list[i]),
                role_str((sensor_role_t)role_list[i]), false);
            continue;
        }
        for (int b = 0; b < 8; b++)
            target_ieee.u8[b] = (uint8_t)bytes[b];

        bool is_sleepy = (type_list[i] == SENSOR_ZG_102Z ||
                          type_list[i] == SENSOR_ZG_102ZA);
        bool joined    = false;

        for (int attempt = 1; attempt <= REJOIN_RETRY_COUNT; attempt++) {
            if (is_sleepy) {
                /* Mgmt_Leave_req with rejoin=true for sleepy sensors */
                if (short_list[i] != 0x0000 && short_list[i] != 0xFFFF) {
                    ezb_zdo_nwk_mgmt_leave_req_t req = {0};
                    req.dst_nwk_addr          = short_list[i];
                    req.field.device_addr     = target_ieee;
                    req.field.remove_children = false;
                    req.field.rejoin          = true;
                    esp_zigbee_lock_acquire(portMAX_DELAY);
                    (void)ezb_zdo_nwk_mgmt_leave_req(&req);
                    esp_zigbee_lock_release();
                }
            } else {
                /* NWK_addr_req broadcast for always-on sensors */
                ezb_zdo_nwk_addr_req_t req = {0};
                req.dst_nwk_addr                  = 0xFFFF;
                req.field.ieee_addr_of_interest    = target_ieee;
                req.field.request_type             = 0;
                req.field.start_index              = 0;
                esp_zigbee_lock_acquire(portMAX_DELAY);
                (void)ezb_zdo_nwk_addr_req(&req);
                esp_zigbee_lock_release();
            }

            PROD_LOG(TAG, "[JOIN] Sensor %d (%s) attempt %d/%d",
                     i + 1, name_list[i], attempt, REJOIN_RETRY_COUNT);
            vTaskDelay(pdMS_TO_TICKS(REJOIN_RETRY_DELAY_MS));

            hub_config_t *cc = lock_config();
            if (cc && cc->sensors[i].online) {
                joined = true;
                sensor_t *s = &cc->sensors[i];
                uart_master_send_sensor_joined(i, s->sensor_name,
                    friendly_name_from_type((sensor_type_t)s->sensor_type),
                    role_str((sensor_role_t)s->sensor_role),
                    true, s->battery_pct);
                unlock_config();
                total_online++;
                break;
            }
            if (cc) unlock_config();
        }

        if (!joined) {
            uart_master_send_sensor_status(i, name_list[i],
                friendly_name_from_type(type_list[i]),
                role_str((sensor_role_t)role_list[i]), false);
            total_offline++;
        }
        vTaskDelay(pdMS_TO_TICKS(REJOIN_POLL_GAP_MS));
    }

    PROD_LOG(TAG, "[JOIN] Complete — online=%d offline=%d",
             total_online, total_offline);
    uart_master_send_sensor_list_complete(count, total_online, total_offline);

    g_rejoin_complete = true;
    vTaskDelete(NULL);
}

// ============================================================================
// ZIGBEE APP SIGNAL HANDLER
// ============================================================================

static void deferred_formation_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    if (!formation_requested) {
        formation_requested = true;
        ezb_bdb_start_top_level_commissioning(
            EZB_BDB_MODE_NETWORK_FORMATION);
    }
    vTaskDelete(NULL);
}

static void hub_ready_task(void *arg)
{
    (void)arg;
    while (!g_rejoin_complete) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    uart_master_send_hub_ready();
    vTaskDelete(NULL);
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
            pairing_window_expired = false;
            pairing_active         = false;

            PROD_LOG(TAG, "Network formed PAN=0x%04hx CH=%d",
                     ezb_nwk_get_panid(), ezb_nwk_get_current_channel());

            g_rejoin_complete = false;
            /*
             * rejoin: 6144 bytes — static arrays + sscanf + ZDO calls
             * hub_ready: 3072 bytes — JSON build + lock chain
             * (was 2048 — crashed every boot; confirmed crash analysis)
             */
            xTaskCreate(rejoin_task,    "rejoin",    6144, NULL, 3, NULL);
            xTaskCreate(hub_ready_task, "hub_ready", 3072, NULL, 3, NULL);
        } else {
            ESP_LOGW(TAG, "Formation failed (status=%d) — retry", (int)status);
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

        /*
         * Send active EP request IMMEDIATELY — no delay.
         *
         * Previous code used a 500ms delay to prevent re-announce loops.
         * Analysis shows re-announce was caused by flooding the sensor with
         * multiple ZCL commands (model ID read + configure reporting + fading
         * time) all at once during the join window — not by the EP request.
         * With only 2 bind requests sent, there is no flooding and no
         * re-announce. Immediate EP request is critical for ZG-102Z which
         * has only ~400ms awake window after DEVICE_ANNCE.
         */
        ezb_zdo_active_ep_req_t req = {0};
        req.dst_nwk_addr               = ann->short_addr;
        req.field.nwk_addr_of_interest  = ann->short_addr;
        req.cb                          = active_ep_callback;
        req.user_ctx = (void *)(uintptr_t)ann->short_addr;
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

            hub_config_t *c = lock_config();
            int total = c ? c->sensor_count : 0;
            if (c) unlock_config();
            uart_master_send_pairing_complete(
                (int)g_new_sensor_count, total);
            g_new_sensor_count = 0;
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
            EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc,
        EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)manufacturer_name);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc,
        EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  (void *)model_identifier);

    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc,
        ezb_zcl_on_off_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER)));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc,
        ezb_zcl_occupancy_sensing_create_cluster_desc(NULL,
            EZB_ZCL_CLUSTER_SERVER)));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc,
        ezb_zcl_ias_zone_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_CLIENT)));

    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));

    ezb_zcl_raw_command_handler_register(raw_frame_handler);
    ezb_zcl_core_action_handler_register(esp_zigbee_zcl_core_action_handler);

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

        {
            hub_config_t *c = lock_config();
            if (c && c->door_closed_pending && c->door_closed_at > 0) {
                time_t elapsed = time(NULL) - c->door_closed_at;
                if (elapsed > DOOR_PENDING_WINDOW_SEC) {
                    PROD_LOG(TAG, "[UNIT] pending expired %lds", (long)elapsed);
                    c->door_closed_pending = false;
                    c->door_closed_at      = 0;
                }
            }
            if (c) unlock_config();
        }

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
// ZIGBEE STACK TASK — passive boot, waits for hub_init
// ============================================================================

static void esp_zigbee_stack_main_task(void *pvParameters)
{
    (void)pvParameters;

    PROD_LOG(TAG, "Waiting for hub_init from Master...");
    while (!g_hub_init_received) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    PROD_LOG(TAG, "hub_init received — starting Zigbee (mode=%s)",
             g_hub_init_mode_debug ? "debug" : "production");

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
    ESP_LOGW(TAG, "FACTORY RESET — erasing NVS");
    nvs_flash_erase();
    nvs_flash_init();
#endif

    g_config.mutex = xSemaphoreCreateMutex();
    if (!g_config.mutex) {
        ESP_LOGE(TAG, "Config mutex failed — halting");
        return;
    }

    memset(&g_config.data, 0, sizeof(g_config.data));
    load_config(&g_config.data);

    esp_err_t uart_err = uart_master_init();
    if (uart_err != ESP_OK) {
        ESP_LOGW(TAG, "UART init failed (%s)", esp_err_to_name(uart_err));
    }

    PROD_LOG(TAG, "v%s ready — waiting for Master ping", FIRMWARE_VERSION);

    xTaskCreate(esp_zigbee_stack_main_task, "Zigbee_main",
                4096 * 2, NULL, 5, NULL);
    xTaskCreate(persist_task,          "persist",    2048, NULL, 3, NULL);
    xTaskCreate(model_id_timeout_task, "id_timeout", 2048, NULL, 2, NULL);

#if WATCHDOG_ENABLE
    xTaskCreate(sensor_watchdog_task, "watchdog",   3072, NULL, 2, NULL);
#else
    PROD_LOG(TAG, "Watchdog disabled");
#endif
}