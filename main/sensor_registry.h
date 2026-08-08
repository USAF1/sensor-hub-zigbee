/*
 * sensor_registry.h — Sensor registry, NVS persistence, and utility helpers
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 */

#ifndef SENSOR_REGISTRY_H
#define SENSOR_REGISTRY_H

#include "main.h"

/* Register a newly-announced device or refresh an existing one. */
void register_or_update(uint16_t short_addr, const char *ieee);

/* Find the sensor index whose short_addr matches, or -1. */
int  find_index_by_short(uint16_t short_addr);

/* Reset all runtime-only meta fields for slot i (does not touch NVS data). */
void reset_meta_runtime(int i);

/* After loading from NVS, restore the parts of g_meta that can be inferred
 * (model_known, model_id string) from the persisted sensor_type. */
void restore_meta_from_nvs(hub_config_t *config);

/* Mark sensor idx as online (updates last_seen, clears miss_count, sends
 * sensor_health if it was offline, recomputes hub aggregate).
 * Must be called with the config lock held. */
void mark_online_locked(hub_config_t *c, int idx);

/* Friendly name table helpers (implemented here, declared in main.h). */

/* FreeRTOS task: periodically flushes g_dirty to NVS.
 * Create with xTaskCreate("persist", 2048, prio 3). */
void persist_task(void *arg);

#endif /* SENSOR_REGISTRY_H */
