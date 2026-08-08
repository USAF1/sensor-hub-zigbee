/*
 * devices/door_zg102z.h — Driver for Tuya ZG-102Z / ZG-102ZA door sensor
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 */

#ifndef DEVICES_DOOR_ZG102Z_H
#define DEVICES_DOOR_ZG102Z_H

#include "sensor_driver.h"

/* Driver instances for ZG-102Z and ZG-102ZA (same logic, different model strings). */
extern const sensor_driver_t door_zg102z_driver;
extern const sensor_driver_t door_zg102za_driver;

#endif /* DEVICES_DOOR_ZG102Z_H */
