/*
 * devices/door_zg102z.c — Driver for Tuya ZG-102Z / ZG-102ZA door sensor
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 *
 * Sensor: Tuya door/window contact sensor, IAS Zone endpoint 1.
 *   IAS Zone alarm_1 bit = contact state (1 = OPEN, 0 = CLOSED).
 *
 * Characteristics:
 *   - Sleepy device: NEVER marked offline by the silence watchdog.
 *   - Battery via PowerConfig cluster attr 0x0021 (halved, 200→100%).
 *   - No Tuya EF00 datapoints; apply_config is a no-op.
 *   - Excluded from hub aggregate (door has no presence).
 *
 * Both model_id variants ("ZG-102Z" and "ZG-102ZA") share this implementation.
 */

#include "door_zg102z.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "main.h"
#include "uart_master.h"

static const char *TAG = "DOOR_ZG102Z";

/* ── on_identified ────────────────────────────────────────────────────────── */

static void door_on_identified(int idx, uint16_t sa, uint8_t ep)
{
    /* Door sensors have no configurable parameters at join.
     * Just log that the device is operational. */
    (void)sa; (void)ep;
    PROD_LOG(TAG, "[ID] ZG-102Z idx=%d OPERATIONAL", idx + 1);
}

/* ── on_ias_status ────────────────────────────────────────────────────────── */

static void door_on_ias_status(int idx, uint16_t zone_status)
{
    bool alarm1 = (zone_status & 0x0001) != 0;
    bool tamper = (zone_status & 0x0004) != 0;
    bool batlow = (zone_status & 0x0008) != 0;

    hub_config_t *c = lock_config();
    if (!c) return;
    sensor_t *s = &c->sensors[idx];
    s->tamper      = tamper;
    s->battery_low = batlow;

    if (s->contact_open == alarm1) {
        /* No change in contact state — nothing to report. */
        unlock_config();
        return;
    }
    s->contact_open   = alarm1;
    s->last_change    = (time_t)(esp_timer_get_time() / 1000000ULL);
    s->door_opened_at = alarm1 ? s->last_change : 0;

    char name[SENSOR_NAME_LEN];
    strncpy(name, s->sensor_name, SENSOR_NAME_LEN - 1);
    name[SENSOR_NAME_LEN - 1] = '\0';

    PROD_LOG(TAG, "[DATA] %s contact=%s", name, alarm1 ? "OPEN" : "CLOSED");
    unlock_config();

    uart_master_send_door(name, alarm1);
    uart_master_notify_door_state(idx, name, alarm1);
    mark_dirty();
}

/* ── on_tuya_dp ───────────────────────────────────────────────────────────── */

static void door_on_tuya_dp(int idx, uint8_t dp, uint8_t dtype,
                             const uint8_t *data, uint16_t dlen)
{
    /* ZG-102Z does not use EF00 datapoints. */
    (void)idx; (void)dp; (void)dtype; (void)data; (void)dlen;
}

/* ── on_report_attr ───────────────────────────────────────────────────────── */

static void door_on_report_attr(int idx, uint16_t cluster, uint16_t attr,
                                 const void *value)
{
#define CLUSTER_POWER_CONFIG 0x0001
#define ATTR_BATTERY_PERCENT 0x0021
    if (cluster == CLUSTER_POWER_CONFIG && attr == ATTR_BATTERY_PERCENT) {
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

static void door_apply_config(int idx, int keep_time_sec, int sensitivity)
{
    /* Door sensor has no configurable parameters. */
    (void)idx; (void)keep_time_sec; (void)sensitivity;
    ESP_LOGW(TAG, "apply_config called on door sensor idx=%d — ignored", idx + 1);
}

/* ── Driver instances ─────────────────────────────────────────────────────── */

const sensor_driver_t door_zg102z_driver = {
    .model_id       = "ZG-102Z",
    .type           = SENSOR_ZG_102Z,
    .role           = ROLE_DOOR,
    .on_identified  = door_on_identified,
    .on_ias_status  = door_on_ias_status,
    .on_tuya_dp     = door_on_tuya_dp,
    .on_report_attr = door_on_report_attr,
    .apply_config   = door_apply_config,
};

const sensor_driver_t door_zg102za_driver = {
    .model_id       = "ZG-102ZA",
    .type           = SENSOR_ZG_102ZA,
    .role           = ROLE_DOOR,
    .on_identified  = door_on_identified,
    .on_ias_status  = door_on_ias_status,
    .on_tuya_dp     = door_on_tuya_dp,
    .on_report_attr = door_on_report_attr,
    .apply_config   = door_apply_config,
};
