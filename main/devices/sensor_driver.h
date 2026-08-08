/*
 * devices/sensor_driver.h — Per-device driver vtable
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 *
 * Each physical sensor family is represented by a sensor_driver_t instance.
 * The central dispatch (zcl_dispatch.c, sensor_identify.c) looks up the
 * right driver at runtime and calls into it; all device-specific logic lives
 * in the driver file, not in the dispatch layer.
 *
 * Registration table and lookup helpers live in sensor_driver.c.
 */

#ifndef DEVICES_SENSOR_DRIVER_H
#define DEVICES_SENSOR_DRIVER_H

#include <stdint.h>
#include "main.h"

/* ── Driver vtable ─────────────────────────────────────────────────────────── */

typedef struct {
    /* Identity */
    const char   *model_id;   /* exact Basic-cluster ModelIdentifier string  */
    sensor_type_t type;
    sensor_role_t role;

    /* Called once after model identification + PowerConfig binding.
     * Use to push initial configuration (keep_time, sensitivity, etc.). */
    void (*on_identified)(int idx, uint16_t short_addr, uint8_t ep);

    /* IAS Zone status-change notification (door: alarm_1 bit = contact open). */
    void (*on_ias_status)(int idx, uint16_t zone_status);

    /* Tuya EF00 datapoint received from the device. */
    void (*on_tuya_dp)(int idx, uint8_t dp, uint8_t dtype,
                       const uint8_t *data, uint16_t dlen);

    /* Standard ZCL attribute report (e.g. battery via PowerConfig 0x0021). */
    void (*on_report_attr)(int idx, uint16_t cluster, uint16_t attr,
                           const void *value);

    /* Push a new configuration to the device.
     * keep_time_sec: desired PIR hold time (seconds).
     * sensitivity:   0=low, 1=medium, 2=high.
     * Pass -1 for a parameter to leave it unchanged.
     * Door drivers may leave this NULL. */
    void (*apply_config)(int idx, int keep_time_sec, int sensitivity);
} sensor_driver_t;

/* ── Lookup ─────────────────────────────────────────────────────────────────── */

/* Return the driver for a given model_id string, or NULL if not found. */
const sensor_driver_t *sensor_driver_find_by_model(const char *model_id);

/* Return the first driver that handles a given sensor_type_t, or NULL. */
const sensor_driver_t *sensor_driver_find(sensor_type_t type);

#endif /* DEVICES_SENSOR_DRIVER_H */
