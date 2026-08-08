/*
 * devices/sensor_driver.c — Driver registration table and lookup helpers
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 *
 * Add a new sensor family by:
 *   1. Creating <model>.h/.c in main/devices/
 *   2. Including the header here and adding to k_drivers[]
 *   3. Adding the .c file to main/CMakeLists.txt SRCS
 */

#include "sensor_driver.h"
#include "pir_zg204zl.h"
#include "door_zg102z.h"

#include <string.h>

static const sensor_driver_t * const k_drivers[] = {
    &pir_zg204zl_driver,
    &door_zg102z_driver,
    &door_zg102za_driver,
};
#define DRIVER_COUNT (sizeof(k_drivers) / sizeof(k_drivers[0]))

const sensor_driver_t *sensor_driver_find_by_model(const char *model_id)
{
    if (!model_id) return NULL;
    for (size_t i = 0; i < DRIVER_COUNT; i++)
        if (strcmp(k_drivers[i]->model_id, model_id) == 0)
            return k_drivers[i];
    return NULL;
}

const sensor_driver_t *sensor_driver_find(sensor_type_t type)
{
    for (size_t i = 0; i < DRIVER_COUNT; i++)
        if (k_drivers[i]->type == type)
            return k_drivers[i];
    return NULL;
}
