/*
 * zcl_dispatch.h — ZCL/Tuya callback dispatch
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 */

#ifndef ZCL_DISPATCH_H
#define ZCL_DISPATCH_H

#include "ezbee/zcl/zcl_core.h"

/* ZCL action callback — registered via ezb_zcl_core_action_handler_register. */
void zcl_action_handler(ezb_zcl_core_action_callback_id_t id, void *msg);

/* ZCL raw-frame callback — registered via ezb_zcl_raw_command_handler_register.
 * Returns false so the stack continues normal processing. */
bool raw_frame_handler(const ezb_zcl_raw_frame_t *raw);

#endif /* ZCL_DISPATCH_H */
