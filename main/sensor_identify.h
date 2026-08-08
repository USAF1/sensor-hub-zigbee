/*
 * sensor_identify.h — Sensor identification: EP request, binding, model-ID read
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 */

#ifndef SENSOR_IDENTIFY_H
#define SENSOR_IDENTIFY_H

#include <stdint.h>
#include "ezbee/zdo/zdo_dev_srv_disc.h"
#include "ezbee/zcl/zcl_general_cmd.h"

/* ZDO active-endpoint callback — binds IAS Zone and triggers model-ID read. */
void active_ep_cb(const ezb_zdo_active_ep_req_result_t *res, void *ctx);

/* Configure PowerConfig binding + battery reporting, then call driver->on_identified. */
void configure_reporting(int idx, uint16_t sa, uint8_t ep);

/* Called by zcl_dispatch on READ_ATTR_RSP for Basic cluster ModelIdentifier. */
void sensor_identify_read_attr_rsp(ezb_zcl_cmd_read_attr_rsp_message_t *m);

/* FreeRTOS task: polls for model-ID read timeouts (sleepy ZG-102Z inferral).
 * Create with xTaskCreate("id_timeout", 2048, prio 2). */
void model_id_timeout_task(void *arg);

#endif /* SENSOR_IDENTIFY_H */
