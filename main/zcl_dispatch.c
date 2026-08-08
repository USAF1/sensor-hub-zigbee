/*
 * zcl_dispatch.c — ZCL/Tuya callback dispatch to device drivers
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 *
 * This module owns the two ZCL entry-points registered with the ezbee stack
 * (zcl_action_handler and raw_frame_handler) and routes each callback to the
 * appropriate device driver based on the sensor's identified type.
 */

#include "zcl_dispatch.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "ezbee/zcl/zcl_core.h"
#include "ezbee/zcl/zcl_general_cmd.h"
#include "ezbee/zcl/cluster/ias_zone.h"

#include "main.h"
#include "sensor_registry.h"
#include "sensor_identify.h"
#include "sensor_driver.h"

static const char *TAG = "ZCL_DISP";

#define CLUSTER_BASIC        0x0000
#define CLUSTER_POWER_CONFIG 0x0001
#define CLUSTER_PRIVATE_TUYA 0xEF00

#define ATTR_BATTERY_PERCENT 0x0021

/* ── IAS Zone enroll ─────────────────────────────────────────────────────── */

static void ias_enroll_handler(ezb_zcl_ias_zone_enroll_req_message_t *m)
{
    if (!m || !m->in.header) return;
    uint16_t sa = m->in.header->src_addr.u.short_addr;
    uint8_t  ep = m->in.header->src_ep;
    int idx = find_index_by_short(sa);
    ezb_zcl_ias_zone_enroll_rsp_cmd_t rsp = {0};
    rsp.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
    rsp.cmd_ctrl.dst_addr.u.short_addr = sa;
    rsp.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
    rsp.cmd_ctrl.dst_ep                = ep;
    rsp.cmd_ctrl.dis_default_rsp       = true;
    rsp.payload.enroll_rsp_code        = EZB_ZCL_IAS_ZONE_ENROLL_RESPONSE_CODE_SUCCESS;
    rsp.payload.zone_id                = (idx >= 0) ? (uint8_t)idx : 0;
    (void)ezb_zcl_ias_zone_enroll_cmd_resp(&rsp);
    if (idx >= 0) g_meta[idx].enroll_sent = true;
}

/* ── IAS Zone status change ──────────────────────────────────────────────── */

static void ias_status_handler(ezb_zcl_ias_zone_status_change_notif_message_t *m)
{
    if (!m || !m->in.header) return;
    uint16_t sa = m->in.header->src_addr.u.short_addr;
    uint16_t zs = m->in.payload.zone_status;
    int idx = find_index_by_short(sa);
    if (idx < 0) return;

    hub_config_t *c = lock_config();
    if (!c) return;
    mark_online_locked(c, idx);

    sensor_type_t t = (sensor_type_t)c->sensors[idx].sensor_type;

    if (t == SENSOR_UNKNOWN) {
        c->sensors[idx].contact_open = (zs & 0x0001) != 0;
        unlock_config();
        return;
    }
    unlock_config();

    /* Dispatch to driver. */
    const sensor_driver_t *drv = sensor_driver_find(t);
    if (drv && drv->on_ias_status)
        drv->on_ias_status(idx, zs);
}

/* ── Attribute report ────────────────────────────────────────────────────── */

static void report_attr_handler(ezb_zcl_cmd_report_attr_message_t *m)
{
    if (!m || !m->in.header || !m->in.variables) return;
    uint16_t sa = m->in.header->src_addr.u.short_addr;
    uint16_t cl = m->info.cluster_id;
    int idx = find_index_by_short(sa);
    if (idx < 0) return;

    hub_config_t *c = lock_config();
    if (!c) return;
    mark_online_locked(c, idx);
    sensor_type_t t = (sensor_type_t)c->sensors[idx].sensor_type;
    unlock_config();

    const sensor_driver_t *drv = sensor_driver_find(t);
    if (!drv) return;

    for (ezb_zcl_report_attr_variable_t *v = m->in.variables; v; v = v->next) {
        if (drv->on_report_attr)
            drv->on_report_attr(idx, cl, v->attr_id, v->attr_value);
    }
}

/* ── ZCL action handler ──────────────────────────────────────────────────── */

void zcl_action_handler(ezb_zcl_core_action_callback_id_t id, void *msg)
{
    switch (id) {
    case EZB_ZCL_CORE_READ_ATTR_RSP_CB_ID:
        sensor_identify_read_attr_rsp((ezb_zcl_cmd_read_attr_rsp_message_t *)msg);
        break;
    case EZB_ZCL_CORE_REPORT_ATTR_CB_ID:
        report_attr_handler((ezb_zcl_cmd_report_attr_message_t *)msg);
        break;
    case EZB_ZCL_CORE_IAS_ZONE_ENROLL_CB_ID:
        ias_enroll_handler((ezb_zcl_ias_zone_enroll_req_message_t *)msg);
        break;
    case EZB_ZCL_CORE_IAS_ZONE_STATUS_CHANGE_NOTIF_CB_ID:
        ias_status_handler((ezb_zcl_ias_zone_status_change_notif_message_t *)msg);
        break;
    default:
        break;
    }
}

/* ── Tuya EF00 raw-frame handler ─────────────────────────────────────────── */

bool raw_frame_handler(const ezb_zcl_raw_frame_t *raw)
{
    if (!raw || !raw->header) return false;
    if (raw->header->cluster_id != CLUSTER_PRIVATE_TUYA) return false;
    if (raw->payload_length < 4 || !raw->payload) return false;

    uint16_t sa = raw->header->src_addr.u.short_addr;
    int idx = find_index_by_short(sa);
    if (idx < 0) return false;

    hub_config_t *c = lock_config();
    if (!c) return false;
    mark_online_locked(c, idx);
    sensor_type_t t = (sensor_type_t)c->sensors[idx].sensor_type;
    unlock_config();

    const sensor_driver_t *drv = sensor_driver_find(t);
    if (!drv || !drv->on_tuya_dp) return false;

    /* Parse the Tuya TLV stream.  Layout:
     *   Byte 0: seq_hi  (Tuya header, skip)
     *   Byte 1: seq_lo
     *   Byte 2: cmd
     *   Then: [dp:1][type:1][len_hi:1][len_lo:1][data:len] ...  */
    const uint8_t *p   = raw->payload;
    uint16_t       len = raw->payload_length;
    uint16_t       i   = 3;   /* skip the 3-byte Tuya header (seq_hi, seq_lo, cmd) */

    while (i + 4 <= len) {
        uint8_t  dp   = p[i];
        uint8_t  type = p[i + 1];
        uint16_t dl   = ((uint16_t)p[i + 2] << 8) | p[i + 3];
        i += 4;
        if (i + dl > len) break;
        drv->on_tuya_dp(idx, dp, type, &p[i], dl);
        i += dl;
    }
    return false;
}
