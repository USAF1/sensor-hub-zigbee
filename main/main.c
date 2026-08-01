/*
 * main.c — Sensor Hub Zigbee Coordinator
 * Innovatsii EMS — Pico 1  |  Firmware 0.2.5
 *
 * Behaviour mirrors Zigbee2MQTT for HOBEIAN/Tuya sensors:
 *
 *   Decode (Tuya EF00 cluster 0xEF00, dataReport cmd 0x02):
 *     ZG-204ZV / ZG-205Z/A  presence  = DP 1  (enum, 1=YES)
 *     ZG-204ZV              temp/hum  = standard clusters 0x0402 / 0x0405
 *     ZG-204ZV / 205        illumin.  = ignored
 *     battery (204ZV/102Z)            = PowerConfig 0x0001 attr 0x0021 (raw/2)
 *     ZG-102Z / ZA          contact   = IAS Zone 0x0500 statusChangeNotification
 *
 *   Configure (Tuya EF00 dataRequest cmd 0x00, DP value type 0x02, 4-byte BE):
 *     fading_time                    = DP 102
 *     motion_detection_sensitivity   = DP 2
 *
 *   Presence is taken ONLY from the Tuya DP (sensor-debounced). IAS Zone
 *   presence for mmWave is ignored — it is the noisy channel that caused
 *   oscillation. mmWave sensors are never pinged.
 *
 *   Rejoin: network is persisted in zb_storage. On boot the coordinator
 *   forms the network only if factory-new, otherwise resumes it and lets
 *   sleepy devices re-announce and rejoin instantly (Z2M behaviour).
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

#ifndef ESP_ZIGBEE_STORAGE_PARTITION_NAME
#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "zb_storage"
#endif

#define TAG "SENSOR_HUB"

/* Production: never wipe NVS on boot — sensors and network must persist. */
#define FACTORY_RESET_MODE  0
#define WATCHDOG_ENABLE     1

#define ZIGBEE_PRIMARY_CHANNEL_MASK   0x07FFF800UL
#define ZIGBEE_SECONDARY_CHANNEL_MASK 0x00000000UL

#define MODEL_ID_TIMEOUT_MS 5000

/* Config defaults written to presence sensors at join. */
#define DEFAULT_FADING_TIME_SEC   30
#define DEFAULT_SENSITIVITY       9

/* ZCL clusters / attributes */
#define CLUSTER_BASIC             0x0000
#define CLUSTER_POWER_CONFIG      0x0001
#define CLUSTER_TEMP_MEASUREMENT  0x0402
#define CLUSTER_HUMIDITY          0x0405
#define CLUSTER_IAS_ZONE          0x0500
#define CLUSTER_PRIVATE_TUYA      0xEF00

#define ATTR_BASIC_ZCL_VERSION      0x0000
#define ATTR_BASIC_MODEL_IDENTIFIER 0x0005
#define ATTR_TEMPERATURE_MEASURED   0x0000
#define ATTR_HUMIDITY_MEASURED      0x0000
#define ATTR_BATTERY_PERCENT        0x0021

/* Tuya EF00 datapoints (confirmed against Z2M) */
#define TUYA_DP_PRESENCE      1     /* enum:  1 = occupied */
#define TUYA_DP_SENSITIVITY   2     /* value: 0..19        */
#define TUYA_DP_FADING_TIME   102   /* value: seconds      */

/* Tuya ZCL commands on cluster 0xEF00 */
#define TUYA_CMD_DATA_REQUEST 0x00  /* coordinator -> device (set) */
#define TUYA_CMD_DATA_REPORT  0x02  /* device -> coordinator       */

/* Tuya DP datatypes */
#define TUYA_TYPE_RAW   0x00
#define TUYA_TYPE_BOOL  0x01
#define TUYA_TYPE_VALUE 0x02
#define TUYA_TYPE_ENUM  0x04

/* Standard IAS Zone bind is required so door sensors enroll. */
static const uint16_t k_bind_clusters[] = { CLUSTER_IAS_ZONE };
#define BIND_COUNT (sizeof(k_bind_clusters) / sizeof(k_bind_clusters[0]))

static void uptime_str(char *buf, size_t len)
{
    uint64_t sec = esp_timer_get_time() / 1000000ULL;
    snprintf(buf, len, "%02u:%02u:%02u",
             (unsigned)(sec / 3600),
             (unsigned)((sec % 3600) / 60),
             (unsigned)(sec % 60));
}

#define PROD_LOG(tag, fmt, ...) do {                 \
    char _ts[10]; uptime_str(_ts, sizeof(_ts));      \
    ESP_LOGI(tag, "[%s] " fmt, _ts, ##__VA_ARGS__);  \
} while (0)

extern void uart_cmd_start_pairing(uint16_t duration_sec);
extern void uart_cmd_stop_pairing(void);
extern void uart_cmd_remove_sensor(int idx);
extern void uart_cmd_factory_reset(void);
extern volatile bool g_hub_init_received;
extern volatile bool g_hub_init_mode_debug;

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

const char *hub_aggregate_str(hub_aggregate_t a)
{
    return a == HUB_AGG_OCCUPIED ? "OCCUPIED" : "VACANT";
}

const char *role_str(sensor_role_t r)
{
    switch (r) {
    case ROLE_DOOR:     return "DOOR";
    case ROLE_PRESENCE: return "PRESENCE";
    default:            return "UNKNOWN";
    }
}

hub_config_safe_t     g_config           = {0};
sensor_runtime_meta_t g_meta[MAX_SENSORS] = {0};
volatile bool         g_watchdog_started  = false;
volatile int          g_new_sensor_count  = 0;
volatile int64_t      g_utc_boot_epoch    = 0;

static bool pairing_active         = false;
static bool pairing_window_expired = false;
static bool network_formed         = false;
static bool formation_requested    = false;
static bool formation_task_started = false;
static volatile bool g_dirty          = false;
static volatile bool g_rejoin_complete = false;
static uint8_t s_tuya_seq = 0;

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

static const sensor_model_def_t *find_model_def(const char *model_id)
{
    for (size_t i = 0; i < sizeof(k_sensor_models) / sizeof(k_sensor_models[0]); i++)
        if (strcmp(k_sensor_models[i].model_id, model_id) == 0)
            return &k_sensor_models[i];
    return NULL;
}

static int find_index_by_short(uint16_t short_addr)
{
    hub_config_t *c = lock_config();
    if (!c) return -1;
    int found = -1;
    for (int i = 0; i < c->sensor_count; i++)
        if (c->sensors[i].short_addr == short_addr) { found = i; break; }
    unlock_config();
    return found;
}

static void reset_meta_runtime(int i)
{
    g_meta[i].miss_count          = 0;
    g_meta[i].reporting_configured = false;
    g_meta[i].fade_sent           = false;
    g_meta[i].bound_once          = false;
    g_meta[i].power_config_bound  = false;
    g_meta[i].enroll_sent         = false;
    g_meta[i].ep_active           = 1;
    g_meta[i].bind_pending        = 0;
    g_meta[i].bind_confirmed      = 0;
    g_meta[i].bind_failed         = 0;
    g_meta[i].model_id_pending    = false;
    g_meta[i].model_id_req_ms     = 0;
    g_meta[i].ping_pending        = false;
    g_meta[i].fade_value          = DEFAULT_FADING_TIME_SEC;
    g_meta[i].sens_value          = DEFAULT_SENSITIVITY;
}

static void restore_meta_from_nvs(hub_config_t *config)
{
    for (int i = 0; i < config->sensor_count; i++) {
        sensor_type_t t = (sensor_type_t)config->sensors[i].sensor_type;
        config->sensors[i].online = false;
        reset_meta_runtime(i);
        if (t != SENSOR_UNKNOWN) {
            g_meta[i].model_known = true;
            for (size_t j = 0; j < sizeof(k_sensor_models) / sizeof(k_sensor_models[0]); j++)
                if (k_sensor_models[j].type == t) {
                    strncpy(g_meta[i].model_id, k_sensor_models[j].model_id,
                            sizeof(g_meta[i].model_id) - 1);
                    break;
                }
        }
    }
}

/* Hub aggregate — OR of online presence sensors. Door excluded. Lock held. */
static void update_hub_aggregate_locked(hub_config_t *c)
{
    bool any = false;
    for (int j = 0; j < c->sensor_count; j++) {
        sensor_type_t t = (sensor_type_t)c->sensors[j].sensor_type;
        if ((t == SENSOR_ZG_204ZV || t == SENSOR_ZG_205Z_A)
            && c->sensors[j].presence && c->sensors[j].online)
            any = true;
    }
    hub_aggregate_t na = any ? HUB_AGG_OCCUPIED : HUB_AGG_VACANT;
    if (na != c->hub_status.aggregate) {
        c->hub_status.aggregate   = na;
        c->hub_status.last_change = time(NULL);
        c->hub_status.timestamp   = time(NULL);
        PROD_LOG(TAG, "[HUB] aggregate=%s", hub_aggregate_str(na));
        uart_master_send_hub_aggregate(hub_aggregate_str(na));
    } else {
        c->hub_status.timestamp = time(NULL);
    }
}

/* ── NVS ─────────────────────────────────────────────────────────────────── */

esp_err_t save_config(hub_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t err = nvs_open("sensor_hub", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    nvs_set_u8(h, "mode",         (uint8_t)config->mode);
    nvs_set_u8(h, "sensor_count", config->sensor_count);
    nvs_set_u8(h, "hub_agg",      (uint8_t)config->hub_status.aggregate);
    for (int i = 0; i < config->sensor_count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "sensor_%d", i);
        nvs_set_blob(h, key, &config->sensors[i], sizeof(sensor_t));
    }
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t load_config(hub_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    memset(g_meta, 0, sizeof(g_meta));

    nvs_handle_t h;
    esp_err_t err = nvs_open("sensor_hub", NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        config->mode                 = MODE_PAIRING;
        config->sensor_count         = 0;
        config->hub_status.aggregate = HUB_AGG_VACANT;
        PROD_LOG(TAG, "NVS empty — first boot");
        return ESP_OK;
    }
    if (err != ESP_OK) return err;

    uint8_t mode = MODE_PAIRING;
    nvs_get_u8(h, "mode", &mode);
    config->mode = (hub_mode_t)mode;
    config->hub_status.aggregate = HUB_AGG_VACANT; /* always start vacant */

    uint8_t count = 0;
    nvs_get_u8(h, "sensor_count", &count);
    if (count > MAX_SENSORS) count = MAX_SENSORS;

    config->sensor_count = 0;
    for (int i = 0; i < count; i++) {
        char key[16]; size_t sz = sizeof(sensor_t);
        snprintf(key, sizeof(key), "sensor_%d", i);
        if (nvs_get_blob(h, key, &config->sensors[i], &sz) == ESP_OK)
            config->sensor_count++;
    }
    nvs_close(h);
    restore_meta_from_nvs(config);
    PROD_LOG(TAG, "Loaded %d sensor(s) from NVS", config->sensor_count);
    return ESP_OK;
}

/* ── Registry ────────────────────────────────────────────────────────────── */

static void register_or_update(uint16_t short_addr, const char *ieee)
{
    hub_config_t *c = lock_config();
    if (!c) return;
    int idx = -1; bool is_new = false;
    for (int i = 0; i < c->sensor_count; i++)
        if (strcmp(c->sensors[i].ieee_addr, ieee) == 0) { idx = i; break; }

    if (idx < 0) {
        if (c->sensor_count >= MAX_SENSORS) { unlock_config(); return; }
        idx = c->sensor_count++; is_new = true;
        memset(&c->sensors[idx], 0, sizeof(sensor_t));
        memset(&g_meta[idx], 0, sizeof(sensor_runtime_meta_t));
        reset_meta_runtime(idx);
    } else {
        /* Re-announce: refresh short addr, keep identity + config. */
        g_meta[idx].reporting_configured = false;
        g_meta[idx].fade_sent            = false;
        g_meta[idx].power_config_bound   = false;
        g_meta[idx].ping_pending         = false;
        g_meta[idx].miss_count           = 0;
        g_meta[idx].model_id_pending     = false;
    }

    c->sensors[idx].short_addr = short_addr;
    strncpy(c->sensors[idx].ieee_addr, ieee, IEEE_ADDR_STR_LEN - 1);
    c->sensors[idx].ieee_addr[IEEE_ADDR_STR_LEN - 1] = '\0';
    c->sensors[idx].endpoint  = 1;
    c->sensors[idx].last_seen = (time_t)(esp_timer_get_time() / 1000000ULL);
    c->sensors[idx].online    = true;
    if (c->sensors[idx].sensor_name[0] == '\0')
        snprintf(c->sensors[idx].sensor_name, SENSOR_NAME_LEN, "Sensor_%d", idx + 1);

    if (is_new && pairing_active) {
        sensor_t *s = &c->sensors[idx];
        char name[SENSOR_NAME_LEN];
        strncpy(name, s->sensor_name, SENSOR_NAME_LEN - 1);
        name[SENSOR_NAME_LEN - 1] = '\0';
        const char *fn = friendly_name_from_type((sensor_type_t)s->sensor_type);
        const char *rl = role_str((sensor_role_t)s->sensor_role);
        unlock_config();
        uart_master_send_new_sensor_joined(idx, name, fn, rl);
        g_new_sensor_count++;
        mark_dirty();
        return;
    }
    unlock_config();
    mark_dirty();
}

/* ── Tuya EF00 write (byte-identical to Z2M) ─────────────────────────────── */
/*
 * Payload layout (matches Z2M manuSpecificTuya.dataRequest):
 *   [frameCtrl=0x11][seq_lo][seq_hi][cmd=0x00][dp][type=0x02][len_hi=0][len_lo=4]
 *   [v24][v16][v8][v0]
 * frameCtrl 0x11 = cluster-specific, direction to server, disable default rsp.
 */
typedef struct { uint16_t sa; uint8_t ep; uint8_t dp; uint32_t val; } tuya_write_t;

static void tuya_write_task(void *arg)
{
    tuya_write_t *w = (tuya_write_t *)arg;
    uint8_t seq = s_tuya_seq++;

    uint8_t payload[12] = {
        0x11, seq, 0x00, TUYA_CMD_DATA_REQUEST,
        w->dp, TUYA_TYPE_VALUE, 0x00, 0x04,
        (uint8_t)((w->val >> 24) & 0xFF),
        (uint8_t)((w->val >> 16) & 0xFF),
        (uint8_t)((w->val >>  8) & 0xFF),
        (uint8_t)((w->val      ) & 0xFF),
    };

    ezb_zcl_custom_cluster_cmd_t cmd = {0};
    cmd.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
    cmd.cmd_ctrl.dst_addr.u.short_addr = w->sa;
    cmd.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
    cmd.cmd_ctrl.dst_ep                = w->ep;
    cmd.cmd_ctrl.cluster_id            = CLUSTER_PRIVATE_TUYA;
    cmd.cmd_ctrl.fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd.cmd_ctrl.fc.dis_default_rsp    = 1;
    cmd.cmd_id                         = TUYA_CMD_DATA_REQUEST;
    cmd.data_length                    = sizeof(payload) - 4; /* frameCtrl..cmd handled by cmd_id; send DP body */
    cmd.data                           = &payload[4];

    esp_zigbee_lock_acquire(portMAX_DELAY);
    (void)ezb_zcl_custom_cluster_cmd_req(&cmd);
    esp_zigbee_lock_release();

    PROD_LOG(TAG, "[CFG] Tuya write DP%u=%lu -> 0x%04hx",
             w->dp, (unsigned long)w->val, w->sa);
    free(w);
    vTaskDelete(NULL);
}

static void tuya_write_dp(uint16_t sa, uint8_t ep, uint8_t dp, uint32_t val)
{
    tuya_write_t *w = malloc(sizeof(tuya_write_t));
    if (!w) return;
    w->sa = sa; w->ep = ep; w->dp = dp; w->val = val;
    xTaskCreate(tuya_write_task, "tuyawr", 3072, w, 3, NULL);
}

/* Public: called by uart_hooks when the Master pushes a new sensitivity/fade. */
void hub_set_sensor_config(int idx, int fading_sec, int sensitivity)
{
    if (idx < 0 || idx >= MAX_SENSORS) return;
    hub_config_t *c = lock_config();
    if (!c || idx >= c->sensor_count) { if (c) unlock_config(); return; }
    sensor_type_t t = (sensor_type_t)c->sensors[idx].sensor_type;
    uint16_t sa = c->sensors[idx].short_addr;
    uint8_t  ep = c->sensors[idx].endpoint ? c->sensors[idx].endpoint : 1;
    unlock_config();

    if (t != SENSOR_ZG_204ZV && t != SENSOR_ZG_205Z_A) return;

    if (fading_sec >= 0 && fading_sec <= 28800) {
        g_meta[idx].fade_value = (uint16_t)fading_sec;
        tuya_write_dp(sa, ep, TUYA_DP_FADING_TIME, (uint32_t)fading_sec);
    }
    if (sensitivity >= 0 && sensitivity <= 19) {
        g_meta[idx].sens_value = (uint8_t)sensitivity;
        tuya_write_dp(sa, ep, TUYA_DP_SENSITIVITY, (uint32_t)sensitivity);
    }
}

/* ── Identification / binding ────────────────────────────────────────────── */

static bool apply_model(int idx, const char *model_id)
{
    const sensor_model_def_t *def = find_model_def(model_id);
    if (!def) { ESP_LOGW(TAG, "Sensor %d unknown model '%s'", idx + 1, model_id); return false; }
    strncpy(g_meta[idx].model_id, model_id, sizeof(g_meta[idx].model_id) - 1);
    g_meta[idx].model_known = true;
    hub_config_t *c = lock_config();
    if (c) {
        c->sensors[idx].sensor_type = (uint8_t)def->type;
        c->sensors[idx].sensor_role = (uint8_t)def->role;
        PROD_LOG(TAG, "[ID] Sensor_%d %s role=%s 0x%04hx",
                 idx + 1, friendly_name_from_type(def->type),
                 role_str(def->role), c->sensors[idx].short_addr);
        unlock_config();
    }
    mark_dirty();
    return true;
}

static void infer_as_door(int idx)
{
    hub_config_t *c = lock_config();
    if (!c) return;
    if ((sensor_type_t)c->sensors[idx].sensor_type != SENSOR_UNKNOWN) { unlock_config(); return; }
    c->sensors[idx].sensor_type = (uint8_t)SENSOR_ZG_102Z;
    c->sensors[idx].sensor_role = (uint8_t)ROLE_DOOR;
    strncpy(g_meta[idx].model_id, "ZG-102Z", sizeof(g_meta[idx].model_id) - 1);
    g_meta[idx].model_known = true;
    PROD_LOG(TAG, "[ID] Sensor_%d inferred ZG-102Z (sleepy door)", idx + 1);
    unlock_config();
    mark_dirty();
}

static void configure_reporting(int idx, uint16_t sa, uint8_t ep)
{
    hub_config_t *c = lock_config();
    if (!c) return;
    sensor_type_t t = (sensor_type_t)c->sensors[idx].sensor_type;
    unlock_config();
    if (t == SENSOR_UNKNOWN) return;

    /* Bind + report battery (PowerConfig) for battery-powered devices. */
    if (!g_meta[idx].power_config_bound) {
        ezb_extaddr_t si, ci;
        ezb_get_extended_address(&ci);
        if (ezb_address_extended_by_short(sa, &si) == EZB_ERR_NONE) {
            ezb_zdo_bind_req_t req = {0};
            req.dst_nwk_addr                 = sa;
            req.field.src_addr               = si;
            req.field.src_ep                 = ep;
            req.field.cluster_id             = CLUSTER_POWER_CONFIG;
            req.field.dst_addr_mode          = EZB_ADDR_MODE_EXT;
            req.field.dst_addr.extended_addr = ci;
            req.field.dst_ep                 = COORDINATOR_ENDPOINT;
            ezb_zdo_bind_req(&req);
            g_meta[idx].power_config_bound = true;
        }
    }

    if (!g_meta[idx].reporting_configured) {
        ezb_zcl_config_report_cmd_t cmd = {0};
        cmd.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
        cmd.cmd_ctrl.dst_addr.u.short_addr = sa;
        cmd.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
        cmd.cmd_ctrl.dst_ep                = ep;
        cmd.cmd_ctrl.fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV;
        cmd.cmd_ctrl.fc.dis_default_rsp    = 1;
        cmd.payload.record_number          = 1;

        ezb_zcl_config_report_record_t batt = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_BATTERY_PERCENT,
            .client    = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT8,
                          .min_interval = 3600, .max_interval = 43200,
                          .reportable_change = {.u8 = 1}},
        };
        cmd.cmd_ctrl.cluster_id  = CLUSTER_POWER_CONFIG;
        cmd.payload.record_field = &batt;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        g_meta[idx].reporting_configured = true;
    }

    /* Push fading + sensitivity to presence sensors (Tuya DP writes). */
    if ((t == SENSOR_ZG_204ZV || t == SENSOR_ZG_205Z_A) && !g_meta[idx].fade_sent) {
        tuya_write_dp(sa, ep, TUYA_DP_FADING_TIME, g_meta[idx].fade_value);
        tuya_write_dp(sa, ep, TUYA_DP_SENSITIVITY, g_meta[idx].sens_value);
        g_meta[idx].fade_sent = true;
    }
    PROD_LOG(TAG, "[ID] %s OPERATIONAL", friendly_name_from_type(t));
}

static void request_model_id(int idx, uint16_t sa, uint8_t ep)
{
    uint16_t attr = ATTR_BASIC_MODEL_IDENTIFIER;
    ezb_zcl_read_attr_cmd_t cmd = {
        .cmd_ctrl = { .dst_addr.addr_mode = EZB_ADDR_MODE_SHORT,
                      .dst_addr.u.short_addr = sa, .src_ep = COORDINATOR_ENDPOINT,
                      .dst_ep = ep, .cluster_id = CLUSTER_BASIC,
                      .fc.direction = EZB_ZCL_CMD_DIRECTION_TO_SRV },
        .payload.attr_number = 1, .payload.attr_field = &attr,
    };
    (void)ezb_zcl_read_attr_cmd_req(&cmd);
    g_meta[idx].model_id_pending = true;
    g_meta[idx].model_id_req_ms  = (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void bind_cb(const ezb_zdp_bind_req_result_t *res, void *ctx)
{
    uintptr_t c = (uintptr_t)ctx;
    int idx = (int)((c >> 16) & 0xFFFF);
    uint16_t sa = (uint16_t)(c & 0xFFFF);
    if (idx < 0 || idx >= MAX_SENSORS) return;
    bool ok = (res && res->error == EZB_ERR_NONE && res->rsp && res->rsp->status == 0);
    if (ok) g_meta[idx].bind_confirmed++; else g_meta[idx].bind_failed++;
    if (g_meta[idx].bind_confirmed + g_meta[idx].bind_failed < g_meta[idx].bind_pending) return;
    g_meta[idx].bound_once = true;
    uint8_t ep = g_meta[idx].ep_active ? g_meta[idx].ep_active : 1;
    if (!g_meta[idx].model_id_pending) request_model_id(idx, sa, ep);
}

static void active_ep_cb(const ezb_zdo_active_ep_req_result_t *res, void *ctx)
{
    uint16_t sa = (uint16_t)(uintptr_t)ctx;
    if (!res || res->error != EZB_ERR_NONE || !res->rsp || res->rsp->active_ep_count == 0) return;
    int idx = find_index_by_short(sa);
    if (idx < 0) return;
    uint8_t ep = res->rsp->active_ep_list[0];
    g_meta[idx].ep_active = ep;

    ezb_extaddr_t si, ci;
    if (ezb_address_extended_by_short(sa, &si) != EZB_ERR_NONE) return;
    ezb_get_extended_address(&ci);
    g_meta[idx].bind_pending = (uint8_t)BIND_COUNT;
    g_meta[idx].bind_confirmed = 0; g_meta[idx].bind_failed = 0;
    for (size_t i = 0; i < BIND_COUNT; i++) {
        ezb_zdo_bind_req_t req = {0};
        req.dst_nwk_addr = sa; req.field.src_addr = si; req.field.src_ep = ep;
        req.field.cluster_id = k_bind_clusters[i];
        req.field.dst_addr_mode = EZB_ADDR_MODE_EXT;
        req.field.dst_addr.extended_addr = ci;
        req.field.dst_ep = COORDINATOR_ENDPOINT;
        req.cb = bind_cb;
        req.user_ctx = (void *)(uintptr_t)(((uint32_t)(uint16_t)idx << 16) | sa);
        ezb_zdo_bind_req(&req);
    }
}

static void model_id_timeout_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
        hub_config_t *c = lock_config();
        int n = c ? c->sensor_count : 0;
        if (c) unlock_config();
        for (int i = 0; i < n; i++) {
            if (!g_meta[i].model_id_pending) continue;
            if (now - g_meta[i].model_id_req_ms < MODEL_ID_TIMEOUT_MS) continue;
            g_meta[i].model_id_pending = false;
            hub_config_t *c2 = lock_config();
            if (!c2) continue;
            uint16_t sa = c2->sensors[i].short_addr;
            uint8_t ep = g_meta[i].ep_active ? g_meta[i].ep_active : 1;
            sensor_type_t t = (sensor_type_t)c2->sensors[i].sensor_type;
            unlock_config();
            if (t == SENSOR_UNKNOWN) { infer_as_door(i); configure_reporting(i, sa, ep); }
        }
    }
}

/* ── RX helpers ──────────────────────────────────────────────────────────── */

static void mark_online_locked(hub_config_t *c, int idx)
{
    bool was_off = !c->sensors[idx].online;
    c->sensors[idx].online = true;
    c->sensors[idx].last_seen = (time_t)(esp_timer_get_time() / 1000000ULL);
    g_meta[idx].ping_pending = false;
    g_meta[idx].miss_count = 0;
    if (was_off) {
        PROD_LOG(TAG, "[WDG] %s ONLINE", c->sensors[idx].sensor_name);
        uart_master_send_sensor_health(c->sensors[idx].sensor_name, true);
        update_hub_aggregate_locked(c);
    }
}

/* Presence from Tuya EF00 DP 1 — the ONLY presence source for mmWave. */
static void handle_presence(int idx, bool occupied)
{
    hub_config_t *c = lock_config();
    if (!c) return;
    mark_online_locked(c, idx);
    sensor_t *s = &c->sensors[idx];
    sensor_type_t t = (sensor_type_t)s->sensor_type;
    if (t != SENSOR_ZG_204ZV && t != SENSOR_ZG_205Z_A) { unlock_config(); return; }
    if (s->presence != occupied) {
        s->presence = occupied;
        s->last_change = (time_t)(esp_timer_get_time() / 1000000ULL);
        char name[SENSOR_NAME_LEN];
        strncpy(name, s->sensor_name, SENSOR_NAME_LEN - 1);
        name[SENSOR_NAME_LEN - 1] = '\0';
        const char *fn = friendly_name_from_type(t);
        PROD_LOG(TAG, "[DATA] %s presence=%s", name, occupied ? "YES" : "NO");
        uart_master_send_sensor_presence(name, fn, occupied);
        update_hub_aggregate_locked(c);
    }
    unlock_config();
    mark_dirty();
}

static uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

/*
 * Raw Tuya EF00 frame. Layout after the ZCL header:
 *   [status][seq_hi][seq_lo][dp][type][len_hi][len_lo][data...]
 * We only act on the presence DP; all other DPs are telemetry/config echoes.
 */
static bool raw_frame_handler(const ezb_zcl_raw_frame_t *raw)
{
    if (!raw || !raw->header) return false;
    if (raw->header->cluster_id != CLUSTER_PRIVATE_TUYA) return false;
    if (raw->payload_length < 7 || !raw->payload) return false;

    uint16_t sa = raw->header->src_addr.u.short_addr;
    int idx = find_index_by_short(sa);
    if (idx < 0) return false;

    const uint8_t *p = raw->payload;
    uint16_t len = raw->payload_length;
    /* Skip 3-byte Tuya header (status + seq16) to first DP. */
    uint16_t i = 3;
    while (i + 4 <= len) {
        uint8_t dp   = p[i];
        uint8_t type = p[i + 1];
        uint16_t dl  = ((uint16_t)p[i + 2] << 8) | p[i + 3];
        i += 4;
        if (i + dl > len) break;

        if (dp == TUYA_DP_PRESENCE && (type == TUYA_TYPE_ENUM || type == TUYA_TYPE_BOOL) && dl >= 1) {
            handle_presence(idx, p[i] != 0);
        } else if (dp == TUYA_DP_FADING_TIME && type == TUYA_TYPE_VALUE && dl == 4) {
            g_meta[idx].fade_value = (uint16_t)be32(&p[i]);
        } else if (dp == TUYA_DP_SENSITIVITY && type == TUYA_TYPE_VALUE && dl == 4) {
            g_meta[idx].sens_value = (uint8_t)be32(&p[i]);
        }
        i += dl;
    }
    return false;
}

static void read_attr_rsp_handler(ezb_zcl_cmd_read_attr_rsp_message_t *m)
{
    if (!m || !m->in.header || m->info.cluster_id != CLUSTER_BASIC) return;
    uint16_t sa = m->in.header->src_addr.u.short_addr;
    uint8_t ep = m->in.header->src_ep;
    int idx = find_index_by_short(sa);
    if (idx < 0) return;
    hub_config_t *c = lock_config();
    if (c) { mark_online_locked(c, idx); unlock_config(); }
    if (!g_meta[idx].model_id_pending) return;
    g_meta[idx].model_id_pending = false;
    bool applied = false;
    for (ezb_zcl_read_attr_rsp_variable_t *v = m->in.variables; v; v = v->next) {
        if (v->status == EZB_ZCL_STATUS_SUCCESS && v->attr_id == ATTR_BASIC_MODEL_IDENTIFIER) {
            uint8_t l = *(uint8_t *)v->attr_value;
            char model[32] = {0};
            if (l >= sizeof(model)) l = sizeof(model) - 1;
            memcpy(model, (char *)(v->attr_value + 1), l);
            applied = apply_model(idx, model);
        }
    }
    if (applied) configure_reporting(idx, sa, g_meta[idx].ep_active ? g_meta[idx].ep_active : (ep ? ep : 1));
}

static void ias_enroll_handler(ezb_zcl_ias_zone_enroll_req_message_t *m)
{
    if (!m || !m->in.header) return;
    uint16_t sa = m->in.header->src_addr.u.short_addr;
    uint8_t ep = m->in.header->src_ep;
    int idx = find_index_by_short(sa);
    ezb_zcl_ias_zone_enroll_rsp_cmd_t rsp = {0};
    rsp.cmd_ctrl.dst_addr.addr_mode = EZB_ADDR_MODE_SHORT;
    rsp.cmd_ctrl.dst_addr.u.short_addr = sa;
    rsp.cmd_ctrl.src_ep = COORDINATOR_ENDPOINT;
    rsp.cmd_ctrl.dst_ep = ep;
    rsp.cmd_ctrl.dis_default_rsp = true;
    rsp.payload.enroll_rsp_code = EZB_ZCL_IAS_ZONE_ENROLL_RESPONSE_CODE_SUCCESS;
    rsp.payload.zone_id = (idx >= 0) ? (uint8_t)idx : 0;
    (void)ezb_zcl_ias_zone_enroll_cmd_resp(&rsp);
    if (idx >= 0) g_meta[idx].enroll_sent = true;
}

/* IAS Zone status — ONLY door sensors. mmWave IAS is ignored (noisy). */
static void ias_status_handler(ezb_zcl_ias_zone_status_change_notif_message_t *m)
{
    if (!m || !m->in.header) return;
    uint16_t sa = m->in.header->src_addr.u.short_addr;
    uint16_t zs = m->in.payload.zone_status;
    bool alarm1 = (zs & 0x0001) != 0;
    bool tamper = (zs & 0x0004) != 0;
    bool batlow = (zs & 0x0008) != 0;
    int idx = find_index_by_short(sa);
    if (idx < 0) return;

    hub_config_t *c = lock_config();
    if (!c) return;
    mark_online_locked(c, idx);
    sensor_t *s = &c->sensors[idx];
    sensor_type_t t = (sensor_type_t)s->sensor_type;
    sensor_role_t r = (sensor_role_t)s->sensor_role;
    s->tamper = tamper; s->battery_low = batlow;

    if (t == SENSOR_UNKNOWN) {
        uint8_t ep = g_meta[idx].ep_active ? g_meta[idx].ep_active : 1;
        s->contact_open = alarm1;
        unlock_config();
        if (!g_meta[idx].model_id_pending) request_model_id(idx, sa, ep);
        return;
    }

    if (r == ROLE_DOOR) {
        if (s->contact_open != alarm1) {
            s->contact_open = alarm1;
            s->last_change = (time_t)(esp_timer_get_time() / 1000000ULL);
            s->door_opened_at = alarm1 ? s->last_change : 0;
            char name[SENSOR_NAME_LEN];
            strncpy(name, s->sensor_name, SENSOR_NAME_LEN - 1);
            name[SENSOR_NAME_LEN - 1] = '\0';
            PROD_LOG(TAG, "[DATA] %s contact=%s", name, alarm1 ? "OPEN" : "CLOSED");
            unlock_config();
            uart_master_send_door(name, alarm1);
            uart_master_notify_door_state(idx, name, alarm1);
            mark_dirty();
            return;
        }
    }
    /* mmWave IAS presence intentionally ignored — presence comes from Tuya DP1. */
    unlock_config();
}

static void report_attr_handler(ezb_zcl_cmd_report_attr_message_t *m)
{
    if (!m || !m->in.header || !m->in.variables) return;
    uint16_t sa = m->in.header->src_addr.u.short_addr;
    uint16_t cl = m->info.cluster_id;
    int idx = find_index_by_short(sa);
    if (idx < 0) return;
    hub_config_t *c = lock_config();
    if (!c) return;
    mark_online_locked(c, idx);
    sensor_t *s = &c->sensors[idx];
    bool changed = false;

    for (ezb_zcl_report_attr_variable_t *v = m->in.variables; v; v = v->next) {
        if (cl == CLUSTER_TEMP_MEASUREMENT && v->attr_id == ATTR_TEMPERATURE_MEASURED) {
            s->temperature_cdeg = *(int16_t *)v->attr_value; changed = true;
            uart_master_send_environment(s->sensor_name,
                (float)s->temperature_cdeg / 100.0f, (float)s->humidity_cpct / 100.0f);
        } else if (cl == CLUSTER_HUMIDITY && v->attr_id == ATTR_HUMIDITY_MEASURED) {
            s->humidity_cpct = *(uint16_t *)v->attr_value; changed = true;
            uart_master_send_environment(s->sensor_name,
                (float)s->temperature_cdeg / 100.0f, (float)s->humidity_cpct / 100.0f);
        } else if (cl == CLUSTER_POWER_CONFIG && v->attr_id == ATTR_BATTERY_PERCENT) {
            uint8_t pct = *(uint8_t *)v->attr_value / 2;
            if (pct > 100) pct = 100;
            s->battery_pct = pct; changed = true;
            uart_master_send_battery(s->sensor_name, pct);
        }
    }
    unlock_config();
    if (changed) mark_dirty();
}

static void zcl_action_handler(ezb_zcl_core_action_callback_id_t id, void *msg)
{
    switch (id) {
    case EZB_ZCL_CORE_READ_ATTR_RSP_CB_ID:
        read_attr_rsp_handler((ezb_zcl_cmd_read_attr_rsp_message_t *)msg); break;
    case EZB_ZCL_CORE_REPORT_ATTR_CB_ID:
        report_attr_handler((ezb_zcl_cmd_report_attr_message_t *)msg); break;
    case EZB_ZCL_CORE_IAS_ZONE_ENROLL_CB_ID:
        ias_enroll_handler((ezb_zcl_ias_zone_enroll_req_message_t *)msg); break;
    case EZB_ZCL_CORE_IAS_ZONE_STATUS_CHANGE_NOTIF_CB_ID:
        ias_status_handler((ezb_zcl_ias_zone_status_change_notif_message_t *)msg); break;
    default: break;
    }
}

/* ── Watchdog: door sensors never pinged; mmWave never pinged (Z2M-style) ── */

#if WATCHDOG_ENABLE
static void watchdog_task(void *arg)
{
    (void)arg;
    uart_hub_config_t cfg;
    while (!g_watchdog_started) vTaskDelay(pdMS_TO_TICKS(1000));
    uart_master_get_config(&cfg);
    PROD_LOG(TAG, "[WDG] started (silence-based, no pinging)");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        uart_master_get_config(&cfg);
        time_t now = (time_t)(esp_timer_get_time() / 1000000ULL);
        hub_config_t *c = lock_config();
        int n = c ? c->sensor_count : 0;
        if (!c) continue;
        for (int i = 0; i < n; i++) {
            time_t last = c->sensors[i].last_seen;
            time_t silence_h = cfg.door_sensor_max_silence_hours;
            if (silence_h == 0) silence_h = 24;
            if (last > 0 && (now - last) > silence_h * 3600 && c->sensors[i].online) {
                char name[SENSOR_NAME_LEN];
                strncpy(name, c->sensors[i].sensor_name, SENSOR_NAME_LEN - 1);
                name[SENSOR_NAME_LEN - 1] = '\0';
                bool presence_type = (c->sensors[i].sensor_type == SENSOR_ZG_204ZV ||
                                      c->sensors[i].sensor_type == SENSOR_ZG_205Z_A);
                if (presence_type) {
                    c->sensors[i].online = false;
                    update_hub_aggregate_locked(c);
                }
                unlock_config();
                uart_master_send_sensor_health(name, false);
                mark_dirty();
                c = lock_config();
                if (!c) break;
            }
        }
        if (c) unlock_config();
    }
}
#endif

/* ── Rejoin / boot ───────────────────────────────────────────────────────── */

static void rejoin_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));
    hub_config_t *c = lock_config();
    int n = c ? c->sensor_count : 0;
    if (c) unlock_config();

    if (n == 0) {
        PROD_LOG(TAG, "[JOIN] No sensors in NVS");
        g_rejoin_complete = true;
        vTaskDelete(NULL);
        return;
    }

    /*
     * Z2M-style rejoin: the network is already restored. Sleepy devices
     * re-announce on their own when they next wake. We simply report the
     * persisted registry to the Master and wait for DEVICE_ANNCE to bring
     * each sensor online. No Leave/steer needed.
     */
    PROD_LOG(TAG, "[JOIN] Network resumed — %d sensor(s) awaiting rejoin", n);
    int online = 0, offline = 0;
    hub_config_t *cc = lock_config();
    if (cc) {
        for (int i = 0; i < cc->sensor_count; i++) {
            sensor_t *s = &cc->sensors[i];
            uart_master_send_sensor_status(i, s->sensor_name,
                friendly_name_from_type((sensor_type_t)s->sensor_type),
                role_str((sensor_role_t)s->sensor_role), s->online);
            if (s->online) online++; else offline++;
        }
        unlock_config();
    }
    uart_master_send_sensor_list_complete(n, online, offline);
    g_rejoin_complete = true;
    vTaskDelete(NULL);
}

static void hub_ready_task(void *arg)
{
    (void)arg;
    while (!g_rejoin_complete) vTaskDelay(pdMS_TO_TICKS(500));
    uart_master_send_hub_ready();
    vTaskDelete(NULL);
}

static void deferred_formation_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    /* Only FORM if factory-new. Otherwise the stack resumes the saved network. */
    if (ezb_bdb_is_factory_new()) {
        if (!formation_requested) {
            formation_requested = true;
            PROD_LOG(TAG, "Factory-new — forming network");
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION);
        }
    } else {
        PROD_LOG(TAG, "Existing network — resuming (instant rejoin enabled)");
        network_formed = true;
        g_rejoin_complete = false;
        xTaskCreate(rejoin_task,    "rejoin",    4096, NULL, 3, NULL);
        xTaskCreate(hub_ready_task, "hub_ready", 3072, NULL, 3, NULL);
    }
    vTaskDelete(NULL);
}

static bool app_signal_handler(const ezb_app_signal_t *sig)
{
    switch (ezb_app_signal_get_type(sig)) {

    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        if (!formation_task_started) {
            formation_task_started = true;
            xTaskCreate(deferred_formation_task, "zb_form", 3072, NULL, 5, NULL);
        }
        break;

    case EZB_BDB_SIGNAL_FORMATION: {
        ezb_bdb_comm_status_t st = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(sig));
        if (st == EZB_BDB_STATUS_SUCCESS) {
            network_formed = true; pairing_window_expired = false; pairing_active = false;
            PROD_LOG(TAG, "Network formed PAN=0x%04hx CH=%d",
                     ezb_nwk_get_panid(), ezb_nwk_get_current_channel());
            g_rejoin_complete = false;
            xTaskCreate(rejoin_task,    "rejoin",    4096, NULL, 3, NULL);
            xTaskCreate(hub_ready_task, "hub_ready", 3072, NULL, 3, NULL);
        } else {
            ESP_LOGW(TAG, "Formation failed (%d) — retry", (int)st);
            formation_requested = false;
        }
        break;
    }

    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        const ezb_zdo_signal_device_annce_params_t *a = ezb_app_signal_get_params(sig);
        if (!a) break;
        char ieee[IEEE_ADDR_STR_LEN] = {0};
        snprintf(ieee, sizeof(ieee), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 a->device_addr.u8[7], a->device_addr.u8[6], a->device_addr.u8[5],
                 a->device_addr.u8[4], a->device_addr.u8[3], a->device_addr.u8[2],
                 a->device_addr.u8[1], a->device_addr.u8[0]);
        PROD_LOG(TAG, "DEVICE_ANNCE 0x%04hx %s", a->short_addr, ieee);
        register_or_update(a->short_addr, ieee);

        ezb_zdo_active_ep_req_t req = {0};
        req.dst_nwk_addr = a->short_addr;
        req.field.nwk_addr_of_interest = a->short_addr;
        req.cb = active_ep_cb;
        req.user_ctx = (void *)(uintptr_t)a->short_addr;
        ezb_zdo_active_ep_req(&req);
        break;
    }

    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t d = *(uint8_t *)ezb_app_signal_get_params(sig);
        pairing_active = (d != 0);
        if (d == 0 && !pairing_window_expired) {
            pairing_window_expired = true;
            hub_config_t *c = lock_config();
            int total = c ? c->sensor_count : 0;
            if (c) unlock_config();
            uart_master_send_pairing_complete((int)g_new_sensor_count, total);
            g_new_sensor_count = 0;
        }
        break;
    }
    default: break;
    }
    return true;
}

static esp_err_t create_coordinator(void)
{
    static const char mfg[] = "Innovatsii EMS";
    static const char mdl[] = "sensor-hub-zigbee";
    ezb_af_device_desc_t dev = ezb_af_create_device_desc();
    ezb_zha_custom_gateway_config_t gc = EZB_ZHA_CUSTOM_GATEWAY_CONFIG();
    ezb_af_ep_desc_t ep = ezb_zha_create_custom_gateway(COORDINATOR_ENDPOINT, &gc);

    ezb_zcl_cluster_desc_t basic = ezb_af_endpoint_get_cluster_desc(ep,
        EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)mfg);
    ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)mdl);

    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep,
        ezb_zcl_ias_zone_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_CLIENT)));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, ep));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev));

    ezb_zcl_raw_command_handler_register(raw_frame_handler);
    ezb_zcl_core_action_handler_register(zcl_action_handler);
    return ESP_OK;
}

static esp_err_t setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(app_signal_handler));
    return ESP_OK;
}

static void persist_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (g_dirty) {
            hub_config_t *c = lock_config();
            if (c) {
                esp_err_t e = save_config(c);
                unlock_config();
                if (e == ESP_OK) g_dirty = false;
            }
        }
    }
}

static void zigbee_main_task(void *arg)
{
    (void)arg;
    PROD_LOG(TAG, "Waiting for hub_init from Master...");
    while (!g_hub_init_received) vTaskDelay(pdMS_TO_TICKS(100));
    PROD_LOG(TAG, "hub_init received — starting Zigbee (%s)",
             g_hub_init_mode_debug ? "debug" : "production");

    esp_zigbee_config_t zc = {0};
    zc.device_config.device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR;
    zc.device_config.install_code_policy = false;
    zc.device_config.zczr_config.max_children = MAX_SENSORS;
    zc.platform_config.storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME;
    zc.platform_config.radio_config.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE;

    ESP_ERROR_CHECK(esp_zigbee_init(&zc));
    ESP_ERROR_CHECK(setup_commissioning());
    ESP_ERROR_CHECK(create_coordinator());
    ESP_ERROR_CHECK(esp_zigbee_start(false)); /* false = resume saved network */
    ESP_ERROR_CHECK(esp_zigbee_launch_mainloop());
    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

void app_main(void)
{
    printf("\nINNOVATSII EMS - SENSOR HUB v%s\n", FIRMWARE_VERSION);
    fflush(stdout);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME));

#if FACTORY_RESET_MODE
    ESP_LOGW(TAG, "FACTORY RESET — erasing NVS");
    nvs_flash_erase();
    nvs_flash_init();
#endif

    g_config.mutex = xSemaphoreCreateMutex();
    if (!g_config.mutex) { ESP_LOGE(TAG, "mutex failed"); return; }

    memset(&g_config.data, 0, sizeof(g_config.data));
    load_config(&g_config.data);

    esp_err_t ue = uart_master_init();
    if (ue != ESP_OK) ESP_LOGW(TAG, "UART init failed (%s)", esp_err_to_name(ue));

    PROD_LOG(TAG, "v%s ready — waiting for Master ping", FIRMWARE_VERSION);

    xTaskCreate(zigbee_main_task,     "Zigbee_main", 8192, NULL, 5, NULL);
    xTaskCreate(persist_task,         "persist",     2048, NULL, 3, NULL);
    xTaskCreate(model_id_timeout_task,"id_timeout",  2048, NULL, 2, NULL);
#if WATCHDOG_ENABLE
    xTaskCreate(watchdog_task,        "watchdog",    3072, NULL, 2, NULL);
#endif
}