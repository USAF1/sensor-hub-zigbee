/*
 * zigbee_core.h — Zigbee coordinator creation, commissioning, and app-signal handler
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 */

#ifndef ZIGBEE_CORE_H
#define ZIGBEE_CORE_H

#include "esp_err.h"

/* FreeRTOS task: waits for hub_init, starts Zigbee stack, runs mainloop.
 * Create with xTaskCreate("Zigbee_main", 8192, prio 5). */
void zigbee_main_task(void *arg);

#endif /* ZIGBEE_CORE_H */
