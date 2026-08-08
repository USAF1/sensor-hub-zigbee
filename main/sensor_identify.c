/*
 * sensor_identify.c — Sensor identification: active-EP, binding, model-ID read
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 *
 * Flow:
 *   DEVICE_ANNCE → register_or_update → active_ep_req
 *     → active_ep_cb  (bind IAS_ZONE)
 *       → bind_cb     (after all binds) → request_model_id
 *         → [device responds] → read_attr_rsp_handler → apply_model
 *                                                      → configure_reporting
 *         → [5 s timeout]    → model_id_timeout_task  → infer_as_door
 *                                                      → configure_reporting
 */

#include "sensor_identify.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "esp_zigbee.h"
#include "ezbee/zdo/zdo_bind_mgmt.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"
#include "ezbee/zcl/zcl_general_cmd.h"
#include "ezbee/zcl/zcl_core.h"

#include "main.h"
#include "sensor_registry.h"
#include "sensor_driver.h"
#include "uart_master.h"

static const char *TAG = "IDENTIFY";

#define CLUSTER_BASIC        0x0000
#define CLUSTER_POWER_CONFIG 0x0001
#define CLUSTER_IAS_ZONE     0x0500

#define ATTR_BASIC_MODEL_IDENTIFIER 0x0005
#define ATTR_BATTERY_PERCENT        0x0021

static void uptime_str(char *buf, size_t len)
{
    uint64_t sec = esp_timer_get_time() / 1000000ULL;
    snprintf(buf, len, "%02u:%02u:%02u",
             (unsigned)(sec / 3600),
             (unsigned)((sec % 3600) / 60),
             (unsigned)(sec % 60));
}

#define PROD_LOG(tag, fmt, ...) do {                 \
    char _ts[10]; uptime_str(_ts, sizeof(_ts));      \
    ESP_LOGI(tag, "[%s] " fmt, _ts, ##__VA_ARGS__);  \
} while (0)

/* IAS Zone is the only cluster bound at join-time for all devices. */
static const uint16_t k_bind_clusters[] = { CLUSTER_IAS_ZONE };
#define BIND_COUNT (sizeof(k_bind_clusters) / sizeof(k_bind_clusters[0]))

/* ── Model-ID read ────────────────────────────────────────────────────────── */

static void request_model_id(int idx, uint16_t sa, uint8_t ep)
{
    uint16_t attr = ATTR_BASIC_MODEL_IDENTIFIER;
    ezb_zcl_read_attr_cmd_t cmd = {
        .cmd_ctrl = {
            .dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT,
            .dst_addr.u.short_addr = sa,
            .src_ep                = COORDINATOR_ENDPOINT,
            .dst_ep                = ep,
            .cluster_id            = CLUSTER_BASIC,
            .fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV,
        },
        .payload.attr_number = 1,
        .payload.attr_field  = &attr,
    };
    (void)ezb_zcl_read_attr_cmd_req(&cmd);
    g_meta[idx].model_id_pending = true;
    g_meta[idx].model_id_req_ms  = (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ── apply_model ──────────────────────────────────────────────────────────── */

static bool apply_model(int idx, const char *model_id)
{
    const sensor_driver_t *drv = sensor_driver_find_by_model(model_id);
    if (!drv) {
        ESP_LOGW(TAG, "Sensor %d unknown model '%s'", idx + 1, model_id);
        return false;
    }
    strncpy(g_meta[idx].model_id, model_id, sizeof(g_meta[idx].model_id) - 1);
    g_meta[idx].model_known = true;
    hub_config_t *c = lock_config();
    if (c) {
        c->sensors[idx].sensor_type = (uint8_t)drv->type;
        c->sensors[idx].sensor_role = (uint8_t)drv->role;
        PROD_LOG(TAG, "[ID] Sensor_%d %s role=%s 0x%04hx",
                 idx + 1, friendly_name_from_type(drv->type),
                 role_str(drv->role), c->sensors[idx].short_addr);
        unlock_config();
    }
    mark_dirty();
    return true;
}

/* ── infer_as_door ────────────────────────────────────────────────────────── */

static void infer_as_door(int idx)
{
    hub_config_t *c = lock_config();
    if (!c) return;
    if ((sensor_type_t)c->sensors[idx].sensor_type != SENSOR_UNKNOWN) {
        unlock_config();
        return;
    }
    c->sensors[idx].sensor_type = (uint8_t)SENSOR_ZG_102Z;
    c->sensors[idx].sensor_role = (uint8_t)ROLE_DOOR;
    strncpy(g_meta[idx].model_id, "ZG-102Z", sizeof(g_meta[idx].model_id) - 1);
    g_meta[idx].model_known = true;
    PROD_LOG(TAG, "[ID] Sensor_%d inferred ZG-102Z (sleepy door)", idx + 1);
    unlock_config();
    mark_dirty();
}

/* ── configure_reporting ─────────────────────────────────────────────────── */

void configure_reporting(int idx, uint16_t sa, uint8_t ep)
{
    hub_config_t *c = lock_config();
    if (!c) return;
    sensor_type_t t = (sensor_type_t)c->sensors[idx].sensor_type;
    unlock_config();
    if (t == SENSOR_UNKNOWN) return;

    /* Bind PowerConfig cluster for battery reporting. */
    if (!g_meta[idx].power_config_bound) {
        ezb_extaddr_t si, ci;
        ezb_get_extended_address(&ci);
        if (ezb_address_extended_by_short(sa, &si) == EZB_ERR_NONE) {
            ezb_zdo_bind_req_t req = {0};
            req.dst_nwk_addr                 = sa;
            req.field.src_addr               = si;
            req.field.src_ep                 = ep;
            req.field.cluster_id             = CLUSTER_POWER_CONFIG;
            req.field.dst_addr_mode          = EZB_ADDR_MODE_EXT;
            req.field.dst_addr.extended_addr = ci;
            req.field.dst_ep                 = COORDINATOR_ENDPOINT;
            ezb_zdo_bind_req(&req);
            g_meta[idx].power_config_bound = true;
        }
    }

    /* Configure battery attribute reporting. */
    if (!g_meta[idx].reporting_configured) {
        ezb_zcl_config_report_cmd_t cmd = {0};
        cmd.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
        cmd.cmd_ctrl.dst_addr.u.short_addr = sa;
        cmd.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
        cmd.cmd_ctrl.dst_ep                = ep;
        cmd.cmd_ctrl.cluster_id            = CLUSTER_POWER_CONFIG;
        cmd.cmd_ctrl.fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV;
        cmd.cmd_ctrl.fc.dis_default_rsp    = 1;
        cmd.payload.record_number          = 1;

        ezb_zcl_config_report_record_t batt = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = ATTR_BATTERY_PERCENT,
            .client    = {
                .attr_type        = EZB_ZCL_ATTR_TYPE_UINT8,
                .min_interval     = 3600,
                .max_interval     = 43200,
                .reportable_change = {.u8 = 1},
            },
        };
        cmd.payload.record_field = &batt;
        (void)ezb_zcl_config_report_cmd_req(&cmd);
        g_meta[idx].reporting_configured = true;
    }

    /* Call driver's on_identified for device-specific initial configuration. */
    const sensor_driver_t *drv = sensor_driver_find(t);
    if (drv && drv->on_identified)
        drv->on_identified(idx, sa, ep);

    PROD_LOG(TAG, "[ID] %s OPERATIONAL", friendly_name_from_type(t));
}

/* ── bind_cb ─────────────────────────────────────────────────────────────── */

static void bind_cb(const ezb_zdp_bind_req_result_t *res, void *ctx)
{
    uintptr_t c = (uintptr_t)ctx;
    int idx = (int)((c >> 16) & 0xFFFF);
    uint16_t sa = (uint16_t)(c & 0xFFFF);
    if (idx < 0 || idx >= MAX_SENSORS) return;
    bool ok = (res && res->error == EZB_ERR_NONE && res->rsp && res->rsp->status == 0);
    if (ok) g_meta[idx].bind_confirmed++; else g_meta[idx].bind_failed++;
    if (g_meta[idx].bind_confirmed + g_meta[idx].bind_failed < g_meta[idx].bind_pending)
        return;
    g_meta[idx].bound_once = true;
    uint8_t ep = g_meta[idx].ep_active ? g_meta[idx].ep_active : 1;
    if (!g_meta[idx].model_id_pending)
        request_model_id(idx, sa, ep);
}

/* ── active_ep_cb ─────────────────────────────────────────────────────────── */

void active_ep_cb(const ezb_zdo_active_ep_req_result_t *res, void *ctx)
{
    uint16_t sa = (uint16_t)(uintptr_t)ctx;
    if (!res || res->error != EZB_ERR_NONE || !res->rsp
        || res->rsp->active_ep_count == 0) return;
    int idx = find_index_by_short(sa);
    if (idx < 0) return;
    uint8_t ep = res->rsp->active_ep_list[0];
    g_meta[idx].ep_active = ep;

    ezb_extaddr_t si, ci;
    if (ezb_address_extended_by_short(sa, &si) != EZB_ERR_NONE) return;
    ezb_get_extended_address(&ci);
    g_meta[idx].bind_pending   = (uint8_t)BIND_COUNT;
    g_meta[idx].bind_confirmed = 0;
    g_meta[idx].bind_failed    = 0;
    for (size_t i = 0; i < BIND_COUNT; i++) {
        ezb_zdo_bind_req_t req = {0};
        req.dst_nwk_addr = sa;
        req.field.src_addr = si;
        req.field.src_ep   = ep;
        req.field.cluster_id = k_bind_clusters[i];
        req.field.dst_addr_mode = EZB_ADDR_MODE_EXT;
        req.field.dst_addr.extended_addr = ci;
        req.field.dst_ep = COORDINATOR_ENDPOINT;
        req.cb       = bind_cb;
        req.user_ctx = (void *)(uintptr_t)(((uint32_t)(uint16_t)idx << 16) | sa);
        ezb_zdo_bind_req(&req);
    }
}

/* ── read_attr_rsp_handler ───────────────────────────────────────────────── */

/* Called by zcl_dispatch when EZB_ZCL_CORE_READ_ATTR_RSP_CB_ID fires. */
void sensor_identify_read_attr_rsp(ezb_zcl_cmd_read_attr_rsp_message_t *m)
{
    if (!m || !m->in.header || m->info.cluster_id != CLUSTER_BASIC) return;
    uint16_t sa = m->in.header->src_addr.u.short_addr;
    uint8_t  ep = m->in.header->src_ep;
    int idx = find_index_by_short(sa);
    if (idx < 0) return;
    hub_config_t *c = lock_config();
    if (c) { mark_online_locked(c, idx); unlock_config(); }
    if (!g_meta[idx].model_id_pending) return;
    g_meta[idx].model_id_pending = false;
    bool applied = false;
    for (ezb_zcl_read_attr_rsp_variable_t *v = m->in.variables; v; v = v->next) {
        if (v->status == EZB_ZCL_STATUS_SUCCESS
            && v->attr_id == ATTR_BASIC_MODEL_IDENTIFIER) {
            uint8_t l = *(uint8_t *)v->attr_value;
            char model[32] = {0};
            if (l >= sizeof(model)) l = (uint8_t)(sizeof(model) - 1);
            memcpy(model, (char *)(v->attr_value + 1), l);
            applied = apply_model(idx, model);
        }
    }
    if (applied)
        configure_reporting(idx, sa,
                            g_meta[idx].ep_active ? g_meta[idx].ep_active
                                                   : (ep ? ep : 1));
}

/* ── model_id_timeout_task ───────────────────────────────────────────────── */

void model_id_timeout_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
        hub_config_t *c = lock_config();
        int n = c ? c->sensor_count : 0;
        if (c) unlock_config();
        for (int i = 0; i < n; i++) {
            if (!g_meta[i].model_id_pending) continue;
            if (now - g_meta[i].model_id_req_ms < MODEL_ID_TIMEOUT_MS) continue;
            g_meta[i].model_id_pending = false;
            hub_config_t *c2 = lock_config();
            if (!c2) continue;
            uint16_t sa = c2->sensors[i].short_addr;
            uint8_t  ep = g_meta[i].ep_active ? g_meta[i].ep_active : 1;
            sensor_type_t t = (sensor_type_t)c2->sensors[i].sensor_type;
            unlock_config();
            if (t == SENSOR_UNKNOWN) {
                infer_as_door(i);
                configure_reporting(i, sa, ep);
            }
        }
    }
}
