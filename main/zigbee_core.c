/*
 * zigbee_core.c — Zigbee coordinator creation, commissioning, and boot tasks
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 *
 * Commissioning strategy (preserving existing "resume via formation" trick):
 *   Always start EZB_BDB_MODE_NETWORK_FORMATION.
 *   - Factory-new: forms a fresh network.
 *   - Existing:    stack resumes it; formation returns FORMATION_FAILURE
 *     (harmless: "already on a network"). Either way the coordinator ends up
 *     commissioned and can accept joins.
 */

#include "zigbee_core.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_zigbee.h"
#include "ezbee/core.h"
#include "ezbee/nwk.h"
#include "ezbee/aps.h"
#include "ezbee/af.h"
#include "ezbee/zdo.h"
#include "ezbee/bdb.h"
#include "ezbee/secur.h"
#include "ezbee/app_signals.h"
#include "ezbee/zcl/zcl_core.h"
#include "ezbee/zcl/zcl_desc.h"
#include "ezbee/zcl/zcl_type.h"
#include "ezbee/zcl/zcl_common.h"
#include "ezbee/zcl/zcl_general_cmd.h"
#include "ezbee/zcl/cluster/ias_zone.h"
#include "ezbee/zdo/zdo_bind_mgmt.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"
#include "ezbee/zdo/zdo_nwk_mgmt.h"

#include "zigbee_gateway.h"

#include "main.h"
#include "sensor_registry.h"
#include "sensor_identify.h"
#include "zcl_dispatch.h"
#include "hub_aggregate.h"
#include "uart_master.h"

#ifndef ESP_ZIGBEE_STORAGE_PARTITION_NAME
#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "zb_storage"
#endif

static const char *TAG = "ZB_CORE";

#define ZIGBEE_PRIMARY_CHANNEL_MASK   0x07FFF800UL
#define ZIGBEE_SECONDARY_CHANNEL_MASK 0x00000000UL
#define WATCHDOG_ENABLE               1

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

/* ── State ──────────────────────────────────────────────────────────────── */

#if WATCHDOG_ENABLE
static void watchdog_task(void *arg);
#endif

static bool pairing_active         = false;
static bool pairing_window_expired = false;
static bool network_formed         = false;
static bool formation_requested    = false;
static bool formation_task_started = false;
static volatile bool g_rejoin_complete = false;

/* Accessor for sensor_registry.c. */
bool zigbee_is_pairing_active(void) { return pairing_active; }

/* ── Boot tasks ─────────────────────────────────────────────────────────── */

static void rejoin_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));
    hub_config_t *c = lock_config();
    int n = c ? c->sensor_count : 0;
    if (c) unlock_config();

    if (n == 0) {
        PROD_LOG(TAG, "[JOIN] No sensors in NVS");
        g_rejoin_complete = true;
        vTaskDelete(NULL);
        return;
    }

    PROD_LOG(TAG, "[JOIN] Network resumed — %d sensor(s) awaiting rejoin", n);
    int online = 0, offline = 0;
    hub_config_t *cc = lock_config();
    if (cc) {
        for (int i = 0; i < cc->sensor_count; i++) {
            sensor_t *s = &cc->sensors[i];
            uart_master_send_sensor_status(i, s->sensor_name,
                friendly_name_from_type((sensor_type_t)s->sensor_type),
                role_str((sensor_role_t)s->sensor_role), s->online);
            if (s->online) online++; else offline++;
        }
        unlock_config();
    }
    uart_master_send_sensor_list_complete(n, online, offline);
    g_rejoin_complete = true;
    vTaskDelete(NULL);
}

static void hub_ready_task(void *arg)
{
    (void)arg;
    while (!g_rejoin_complete) vTaskDelay(pdMS_TO_TICKS(500));
    uart_master_send_hub_ready();
    vTaskDelete(NULL);
}

static void start_ready_tasks_once(void)
{
    static bool started = false;
    if (started) return;
    started = true;
    g_rejoin_complete = false;
    xTaskCreate(rejoin_task,    "rejoin",    4096, NULL, 3, NULL);
    xTaskCreate(hub_ready_task, "hub_ready", 3072, NULL, 3, NULL);
#if WATCHDOG_ENABLE
    xTaskCreate(watchdog_task,  "watchdog",  3072, NULL, 2, NULL);
#endif
}

/* ── Network formation ──────────────────────────────────────────────────── */

static void deferred_formation_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    if (!formation_requested) {
        formation_requested = true;
        if (ezb_bdb_is_factory_new())
            PROD_LOG(TAG, "Factory-new — forming network");
        else
            PROD_LOG(TAG, "Existing network — resuming via BDB commissioning");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION);
    }
    vTaskDelete(NULL);
}

/* ── App-signal handler ──────────────────────────────────────────────────── */

static bool app_signal_handler(const ezb_app_signal_t *sig)
{
    switch (ezb_app_signal_get_type(sig)) {

    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        if (!formation_task_started) {
            formation_task_started = true;
            xTaskCreate(deferred_formation_task, "zb_form", 3072, NULL, 5, NULL);
        }
        break;

    case EZB_BDB_SIGNAL_FORMATION: {
        ezb_bdb_comm_status_t st =
            *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(sig));
        network_formed = true;
        pairing_window_expired = false;
        pairing_active = false;
        if (st == EZB_BDB_STATUS_SUCCESS) {
            PROD_LOG(TAG, "Network formed PAN=0x%04hx CH=%d",
                     ezb_nwk_get_panid(), ezb_nwk_get_current_channel());
        } else {
            PROD_LOG(TAG, "Formation returned %d — resumed existing network "
                         "PAN=0x%04hx CH=%d", (int)st,
                     ezb_nwk_get_panid(), ezb_nwk_get_current_channel());
        }
        start_ready_tasks_once();
        break;
    }

    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        const ezb_zdo_signal_device_annce_params_t *a = ezb_app_signal_get_params(sig);
        if (!a) break;
        char ieee[IEEE_ADDR_STR_LEN] = {0};
        snprintf(ieee, sizeof(ieee),
                 "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 a->device_addr.u8[7], a->device_addr.u8[6],
                 a->device_addr.u8[5], a->device_addr.u8[4],
                 a->device_addr.u8[3], a->device_addr.u8[2],
                 a->device_addr.u8[1], a->device_addr.u8[0]);
        PROD_LOG(TAG, "DEVICE_ANNCE 0x%04hx %s", a->short_addr, ieee);
        register_or_update(a->short_addr, ieee);

        ezb_zdo_active_ep_req_t req = {0};
        req.dst_nwk_addr = a->short_addr;
        req.field.nwk_addr_of_interest = a->short_addr;
        req.cb       = active_ep_cb;
        req.user_ctx = (void *)(uintptr_t)a->short_addr;
        ezb_zdo_active_ep_req(&req);
        break;
    }

    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t d = *(uint8_t *)ezb_app_signal_get_params(sig);
        pairing_active = (d != 0);
        if (d == 0 && !pairing_window_expired) {
            pairing_window_expired = true;
            hub_config_t *c = lock_config();
            int total = c ? c->sensor_count : 0;
            if (c) unlock_config();
            uart_master_send_pairing_complete((int)g_new_sensor_count, total);
            g_new_sensor_count = 0;
        }
        break;
    }
    default: break;
    }
    return true;
}

/* ── Watchdog ────────────────────────────────────────────────────────────── */

#if WATCHDOG_ENABLE
static void watchdog_task(void *arg)
{
    (void)arg;
    uart_hub_config_t cfg;
    while (!g_watchdog_started) vTaskDelay(pdMS_TO_TICKS(1000));
    uart_master_get_config(&cfg);
    PROD_LOG(TAG, "[WDG] started (silence-based)");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        uart_master_get_config(&cfg);
        time_t now = (time_t)(esp_timer_get_time() / 1000000ULL);
        hub_config_t *c = lock_config();
        int n = c ? c->sensor_count : 0;
        if (!c) continue;
        for (int i = 0; i < n; i++) {
            /* Only ZG-204ZL PIR sensors are marked offline by the watchdog.
             * Door sensors (ZG-102Z/ZA) are sleepy — never silenced offline. */
            sensor_type_t t = (sensor_type_t)c->sensors[i].sensor_type;
            if (t != SENSOR_ZG_204ZL) continue;

            time_t last = c->sensors[i].last_seen;
            /* door_sensor_max_silence_hours is reused here as the general "max
             * silence before offline" threshold for all active (PIR) sensors.
             * The UART protocol field name is preserved for Master compatibility. */
            time_t silence_h = cfg.door_sensor_max_silence_hours;
            if (silence_h == 0) silence_h = 24;
            if (last > 0 && (now - last) > silence_h * 3600
                && c->sensors[i].online) {
                c->sensors[i].online = false;
                update_hub_aggregate_locked(c);
                char name[SENSOR_NAME_LEN];
                strncpy(name, c->sensors[i].sensor_name, SENSOR_NAME_LEN - 1);
                name[SENSOR_NAME_LEN - 1] = '\0';
                unlock_config();
                uart_master_send_sensor_health(name, false);
                mark_dirty();
                c = lock_config();
                if (!c) break;
            }
        }
        if (c) unlock_config();
    }
}
#endif

/* ── Coordinator creation ────────────────────────────────────────────────── */

static esp_err_t create_coordinator(void)
{
    static const char mfg[] = "Innovatsii EMS";
    static const char mdl[] = "sensor-hub-zigbee";
    ezb_af_device_desc_t dev = ezb_af_create_device_desc();
    ezb_zha_custom_gateway_config_t gc = EZB_ZHA_CUSTOM_GATEWAY_CONFIG();
    ezb_af_ep_desc_t ep = ezb_zha_create_custom_gateway(COORDINATOR_ENDPOINT, &gc);

    ezb_zcl_cluster_desc_t basic = ezb_af_endpoint_get_cluster_desc(ep,
        EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic,
        EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)mfg);
    ezb_zcl_basic_cluster_desc_add_attr(basic,
        EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)mdl);

    /* IAS Zone CLIENT cluster on the coordinator endpoint — required to receive
     * zone enroll requests and status change notifications from door sensors. */
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep,
        ezb_zcl_ias_zone_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_CLIENT)));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, ep));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev));

    ezb_zcl_raw_command_handler_register(raw_frame_handler);
    ezb_zcl_core_action_handler_register(zcl_action_handler);
    return ESP_OK;
}

static esp_err_t setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(app_signal_handler));
    return ESP_OK;
}

/* ── Main Zigbee task ────────────────────────────────────────────────────── */

extern volatile bool g_hub_init_received;    /* defined in uart_master.c */
extern volatile bool g_hub_init_mode_debug;  /* defined in uart_master.c */

void zigbee_main_task(void *arg)
{
    (void)arg;
    PROD_LOG(TAG, "Waiting for hub_init from Master...");
    while (!g_hub_init_received) vTaskDelay(pdMS_TO_TICKS(100));
    PROD_LOG(TAG, "hub_init received — starting Zigbee (%s)",
             g_hub_init_mode_debug ? "debug" : "production");

    esp_zigbee_config_t zc = {0};
    zc.device_config.device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR;
    zc.device_config.install_code_policy = false;
    zc.device_config.zczr_config.max_children = MAX_SENSORS;
    zc.platform_config.storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME;
    zc.platform_config.radio_config.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE;

    ESP_ERROR_CHECK(esp_zigbee_init(&zc));
    ESP_ERROR_CHECK(setup_commissioning());
    ESP_ERROR_CHECK(create_coordinator());
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    ESP_ERROR_CHECK(esp_zigbee_launch_mainloop());
    esp_zigbee_deinit();
    vTaskDelete(NULL);
}
