/*
 * hub_aggregate.h — Hub presence aggregate (OR of online PIR sensors)
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 */

#ifndef HUB_AGGREGATE_H
#define HUB_AGGREGATE_H

#include "main.h"

/* Recompute hub_aggregate from the current sensor set.
 * Aggregate = OCCUPIED if ANY online PRESENCE sensor reports presence.
 * Door sensors are excluded.
 * Must be called with the config lock held. */
void update_hub_aggregate_locked(hub_config_t *c);

#endif /* HUB_AGGREGATE_H */
