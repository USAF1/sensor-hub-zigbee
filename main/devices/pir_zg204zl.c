/*
 * devices/pir_zg204zl.c — Driver for HOBEIAN ZG-204ZL PIR sensor
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 *
 * Sensor: "Luminance motion sensor" (Tuya / HOBEIAN)
 *   Zigbee model: ZG-204ZL
 *   Cluster:      Tuya EF00 (private), endpoint 1
 *
 * Datapoint map (verified from Z2M herdsman debug capture):
 *
 *   DP  | Tuya type | Meaning             | Encoding
 *   ----+-----------+---------------------+----------------------------------
 *    1  | enum/bool | occupancy (motion)  | 1 byte; non-zero = motion present
 *    4  | value 4B  | battery percent     | big-endian uint32, raw 0..100
 *    9  | enum 1B   | sensitivity         | 0=low, 1=medium, 2=high
 *   10  | enum 1B   | keep_time (PIR hold)| 0=10s, 1=30s, 2=60s, 3=120s
 *       |           |   Z2M: set {keep_time:"10"} → dp:10, datatype:4, data:[0]
 *   12  | value     | illuminance         | DISCARD
 *  102  | value     | illuminance_interval| DISCARD
 *  103  | value     | unknown             | DISCARD
 *
 * Battery ALSO arrives via genPowerCfg cluster attr 0x0021 (halved, 200→100%).
 *
 * Occupancy polarity: PIR_OCCUPANCY_ACTIVE_VALUE is 1 (non-zero = occupied).
 * Change to 0 if field testing shows inverted polarity.
 *
 * Config write uses ENUM datatype (0x04) with 1-byte payload for DP9/DP10,
 * confirmed by Z2M capture (datatype:4).
 */

#include "pir_zg204zl.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "main.h"
#include "hub_aggregate.h"
#include "tuya_ef00.h"
#include "uart_master.h"

static const char *TAG = "PIR_ZG204ZL";

/* Polarity of DP1: non-zero value from the device means "occupied".
 * Verified: device sends 1 for motion, 0 for clear. Change to 0 if inverted. */
#define PIR_OCCUPANCY_ACTIVE_VALUE  1

/* Valid keep_time seconds in order of their DP10 enum codes 0..3. */
static const uint16_t k_keep_time_sec[4] = {10, 30, 60, 120};

/* Map keep_time seconds → nearest DP10 enum (0..3).
 * Clamps to valid range; rounds to nearest step. */
static uint8_t keep_time_to_enum(int sec)
{
    if (sec <= 10)  return 0;
    if (sec <= 30)  return 1;
    if (sec <= 60)  return 2;
    return 3;           /* >= 120 */
}

/* ── on_identified ────────────────────────────────────────────────────────── */

static void pir_on_identified(int idx, uint16_t sa, uint8_t ep)
{
    if (g_meta[idx].config_sent) return;

    /* Read defaults from UART config (global presence fading / sensitivity). */
    uart_hub_config_t cfg;
    uart_master_get_config(&cfg);

    int keep_sec  = (cfg.presence_fading_time_sec > 0)
                    ? (int)cfg.presence_fading_time_sec
                    : PIR_KEEP_TIME_DEFAULT_SEC;
    int sens      = (int)cfg.motion_sensitivity;
    if (sens > 2) sens = 2;   /* PIR sensitivity is 0..2 only */

    uint8_t kt_enum = keep_time_to_enum(keep_sec);
    tuya_write_dp_enum(sa, ep, 10, kt_enum);
    tuya_write_dp_enum(sa, ep, 9,  (uint8_t)sens);

    g_meta[idx].keep_time_sec = k_keep_time_sec[kt_enum];
    g_meta[idx].sensitivity   = (uint8_t)sens;
    g_meta[idx].config_sent   = true;

    PROD_LOG(TAG, "[CFG] ZG-204ZL idx=%d keep=%us(enum %u) sens=%d",
             idx + 1, (unsigned)k_keep_time_sec[kt_enum], kt_enum, sens);
}

/* ── on_ias_status ────────────────────────────────────────────────────────── */

static void pir_on_ias_status(int idx, uint16_t zone_status)
{
    /* The ZG-204ZL also fires IAS Zone notifications but the authoritative
     * occupancy is DP1.  We just mark the sensor online here — no presence
     * update from IAS.  mark_online_locked has already been called by the
     * dispatch layer before invoking this callback. */
    (void)idx;
    (void)zone_status;
}

/* ── on_tuya_dp ───────────────────────────────────────────────────────────── */

static void pir_on_tuya_dp(int idx, uint8_t dp, uint8_t dtype,
                            const uint8_t *data, uint16_t dlen)
{
    hub_config_t *c = lock_config();
    if (!c) return;
    sensor_t *s = &c->sensors[idx];

    switch (dp) {
    case 1: {
        /* DP1 — occupancy (enum or bool, 1 byte).
         * Non-zero means motion/presence active. */
        if (dlen < 1) { unlock_config(); return; }
        bool occupied = (data[0] != 0) == (PIR_OCCUPANCY_ACTIVE_VALUE != 0);
        if (s->presence != occupied) {
            s->presence = occupied;
            s->last_change = (time_t)(esp_timer_get_time() / 1000000ULL);
            char name[SENSOR_NAME_LEN];
            strncpy(name, s->sensor_name, SENSOR_NAME_LEN - 1);
            name[SENSOR_NAME_LEN - 1] = '\0';
            const char *fn = friendly_name_from_type((sensor_type_t)s->sensor_type);
            PROD_LOG(TAG, "[DATA] %s presence=%s", name, occupied ? "YES" : "NO");
            update_hub_aggregate_locked(c);
            unlock_config();
            uart_master_send_sensor_presence(name, fn, occupied);
            mark_dirty();
            return;
        }
        break;
    }

    case 4: {
        /* DP4 — battery percent (value, 4-byte big-endian uint32, raw 0..100). */
        if (dlen < 4) { unlock_config(); return; }
        uint32_t batt_raw = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16)
                          | ((uint32_t)data[2] <<  8) |  (uint32_t)data[3];
        uint8_t pct = (batt_raw > 100u) ? 100u : (uint8_t)batt_raw;
        bool changed = (s->battery_pct != pct);
        s->battery_pct = pct;
        char name[SENSOR_NAME_LEN];
        strncpy(name, s->sensor_name, SENSOR_NAME_LEN - 1);
        name[SENSOR_NAME_LEN - 1] = '\0';
        unlock_config();
        uart_master_send_battery(name, pct);
        if (changed) mark_dirty();
        return;
    }

    case 9:
        /* DP9 — sensitivity enum (0=low, 1=medium, 2=high). */
        if (dlen >= 1) {
            uint8_t sens = data[0];
            if (sens > 2) sens = 2;
            g_meta[idx].sensitivity = sens;
        }
        break;

    case 10:
        /* DP10 — keep_time enum (0=10s, 1=30s, 2=60s, 3=120s). */
        if (dlen >= 1) {
            uint8_t kt = data[0];
            if (kt > 3) kt = 3;
            g_meta[idx].keep_time_sec = k_keep_time_sec[kt];
        }
        break;

    case 12:
    case 102:
    case 103:
        /* Illuminance, illuminance_interval, unknown — discard. */
        break;

    default:
        ESP_LOGD(TAG, "ZG-204ZL idx=%d unknown DP%u dtype=0x%02x dlen=%u",
                 idx + 1, dp, dtype, dlen);
        break;
    }

    unlock_config();
}

/* ── on_report_attr ───────────────────────────────────────────────────────── */

static void pir_on_report_attr(int idx, uint16_t cluster, uint16_t attr,
                                const void *value)
{
#define CLUSTER_POWER_CONFIG 0x0001
#define ATTR_BATTERY_PERCENT 0x0021
    if (cluster == CLUSTER_POWER_CONFIG && attr == ATTR_BATTERY_PERCENT) {
        /* batteryPercentageRemaining: value 200 = 100 %; halve it. */
        uint8_t raw = *(const uint8_t *)value;
        uint8_t pct = raw / 2;
        if (pct > 100) pct = 100;
        hub_config_t *c = lock_config();
        if (!c) return;
        bool changed = (c->sensors[idx].battery_pct != pct);
        c->sensors[idx].battery_pct = pct;
        char name[SENSOR_NAME_LEN];
        strncpy(name, c->sensors[idx].sensor_name, SENSOR_NAME_LEN - 1);
        name[SENSOR_NAME_LEN - 1] = '\0';
        unlock_config();
        uart_master_send_battery(name, pct);
        if (changed) mark_dirty();
    }
#undef CLUSTER_POWER_CONFIG
#undef ATTR_BATTERY_PERCENT
}

/* ── apply_config ─────────────────────────────────────────────────────────── */

static void pir_apply_config(int idx, int keep_time_sec, int sensitivity)
{
    hub_config_t *c = lock_config();
    if (!c || idx >= c->sensor_count) { if (c) unlock_config(); return; }
    uint16_t sa = c->sensors[idx].short_addr;
    uint8_t  ep = c->sensors[idx].endpoint ? c->sensors[idx].endpoint : 1;
    unlock_config();

    if (keep_time_sec >= 0) {
        uint8_t kt_enum = keep_time_to_enum(keep_time_sec);
        tuya_write_dp_enum(sa, ep, 10, kt_enum);
        g_meta[idx].keep_time_sec = k_keep_time_sec[kt_enum];
        PROD_LOG(TAG, "[CFG] idx=%d DP10 keep_time %ds → enum %u",
                 idx + 1, keep_time_sec, kt_enum);
    }
    if (sensitivity >= 0) {
        uint8_t sens = (sensitivity > 2) ? 2 : (uint8_t)sensitivity;
        tuya_write_dp_enum(sa, ep, 9, sens);
        g_meta[idx].sensitivity = sens;
        PROD_LOG(TAG, "[CFG] idx=%d DP9 sensitivity=%u", idx + 1, sens);
    }
}

/* ── Driver instance ─────────────────────────────────────────────────────── */

const sensor_driver_t pir_zg204zl_driver = {
    .model_id       = "ZG-204ZL",
    .type           = SENSOR_ZG_204ZL,
    .role           = ROLE_PRESENCE,
    .on_identified  = pir_on_identified,
    .on_ias_status  = pir_on_ias_status,
    .on_tuya_dp     = pir_on_tuya_dp,
    .on_report_attr = pir_on_report_attr,
    .apply_config   = pir_apply_config,
};
