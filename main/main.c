/*
 * Sensor Hub Zigbee Coordinator
 *
 * Strict per-model build
 * - Exact model classification:
 *   - ZG-204ZV
 *   - ZG-205Z/A
 *   - ZG-102Z
 *   - ZG-102ZA
 * - One parser per model
 * - One source per signal
 * - No mixed family fallbacks
 * - Presence/temperature/battery only from the correct cluster per model
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

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
#include "ezbee/zdo/zdo_bind_mgmt.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"
#include "ezbee/zdo/zdo_nwk_mgmt.h"

#include "main.h"
#include "zigbee_gateway.h"

#ifndef ESP_ZIGBEE_STORAGE_PARTITION_NAME
#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "zb_storage"
#endif

#define TAG "SENSOR_HUB"

#define FACTORY_RESET_MODE 0
#define PERSIST_SENSORS_MODE 0
#define RAW_LOGS_MODE 1

#define PAIRING_DURATION_SEC 120

#define ZIGBEE_PRIMARY_CHANNEL_MASK   0x07FFF800
#define ZIGBEE_SECONDARY_CHANNEL_MASK 0x00000000

#define CLUSTER_BASIC             0x0000
#define CLUSTER_POWER_CONFIG      0x0001
#define CLUSTER_TEMP_MEASUREMENT   0x0402
#define CLUSTER_HUMIDITY          0x0405
#define CLUSTER_OCCUPANCY_SENSING  0x0406
#define CLUSTER_IAS_ZONE           0x0500
#define CLUSTER_ILLUMINANCE       0x0400
#define CLUSTER_PRIVATE_TUYA       0xEF00

#define ATTR_BASIC_MANUFACTURER_NAME 0x0004
#define ATTR_BASIC_MODEL_IDENTIFIER  0x0005

#define ATTR_OCCUPANCY              0x0000
#define ATTR_TEMPERATURE_MEASURED   0x0000
#define ATTR_HUMIDITY_MEASURED      0x0000
#define ATTR_ILLUMINANCE_MEASURED   0x0000
#define ATTR_BATTERY_PERCENT        0x0021

#define TUYA_DP_FADING_TIME         0x66

#if RAW_LOGS_MODE
#define RAW_LOG(...) printf(__VA_ARGS__)
#else
#define RAW_LOG(...) do { } while (0)
#endif

typedef enum {
    SENSOR_UNKNOWN = 0,
    SENSOR_ZG_204ZV,
    SENSOR_ZG_205Z_A,
    SENSOR_ZG_102Z,
    SENSOR_ZG_102ZA,
} sensor_type_t;

typedef struct {
    const char *model_id;
    sensor_type_t type;
} sensor_model_def_t;

typedef struct {
    sensor_type_t type;
    bool model_known;
    char model_id[32];
    bool bound_once;
    bool reporting_configured;
    bool fade_sent;
} sensor_runtime_meta_t;

static const sensor_model_def_t k_sensor_models[] = {
    {"ZG-204ZV", SENSOR_ZG_204ZV},
    {"ZG-205Z/A", SENSOR_ZG_205Z_A},
    {"ZG-102Z", SENSOR_ZG_102Z},
    {"ZG-102ZA", SENSOR_ZG_102ZA},
};

hub_config_safe_t g_config = {0};
static sensor_runtime_meta_t g_meta[MAX_SENSORS] = {0};

static bool pairing_active = false;
static bool pairing_window_expired = false;
static bool network_formed = false;
static bool formation_requested = false;
static bool formation_task_started = false;
static bool zigbee_ready = false;
static bool g_dirty = false;

/* Forward declarations */
static void print_sensor_summary(void);
esp_err_t save_config(hub_config_t *config);
esp_err_t load_config(hub_config_t *config);
static void handle_zg102z_raw(uint16_t src_addr, uint16_t cluster_id, const uint8_t *payload, size_t len);
static void handle_zg102za_raw(uint16_t src_addr, uint16_t cluster_id, const uint8_t *payload, size_t len);

hub_config_t *lock_config(void)
{
    if (!g_config.mutex) return NULL;
    if (xSemaphoreTake(g_config.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) return &g_config.data;
    ESP_LOGW(TAG, "lock_config timeout");
    return NULL;
}

void unlock_config(void)
{
    if (g_config.mutex) xSemaphoreGive(g_config.mutex);
}

static void mark_dirty(void) { g_dirty = true; }

static const sensor_model_def_t *find_model_def(const char *model_id)
{
    for (size_t i = 0; i < sizeof(k_sensor_models) / sizeof(k_sensor_models[0]); i++) {
        if (strcmp(k_sensor_models[i].model_id, model_id) == 0) return &k_sensor_models[i];
    }
    return NULL;
}

static int find_sensor_index_by_short(uint16_t short_addr)
{
    hub_config_t *c = lock_config();
    if (!c) return -1;
    for (int i = 0; i < c->sensor_count; i++) {
        if (c->sensors[i].short_addr == short_addr) {
            unlock_config();
            return i;
        }
    }
    unlock_config();
    return -1;
}

static void set_default_sensor_name(sensor_t *s, int idx)
{
    if (s->sensor_name[0] == '\0') snprintf(s->sensor_name, SENSOR_NAME_LEN, "Sensor_%d", idx + 1);
}

static void apply_model_to_sensor(int idx, const char *model_id)
{
    const sensor_model_def_t *def = find_model_def(model_id);
    if (!def) {
        g_meta[idx].type = SENSOR_UNKNOWN;
        g_meta[idx].model_known = false;
        return;
    }

    strncpy(g_meta[idx].model_id, model_id, sizeof(g_meta[idx].model_id) - 1);
    g_meta[idx].model_id[sizeof(g_meta[idx].model_id) - 1] = '\0';
    g_meta[idx].type = def->type;
    g_meta[idx].model_known = true;
    ESP_LOGI(TAG, "Sensor %d model: %s", idx + 1, g_meta[idx].model_id);
}

static void print_banner(void)
{
    printf("\n\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║       🏠 INNOVATSII EMS - SENSOR HUB 🏠                      ║\n");
    printf("║       Strict Per-Model Build                                 ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    fflush(stdout);
}

static void print_sensor_summary(void)
{
    hub_config_t *c = lock_config();
    if (!c) return;

    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║ SENSOR REGISTRY (%d/%d)\n", c->sensor_count, MAX_SENSORS);
    printf("║ Network: %s | Pairing: %s\n", network_formed ? "ACTIVE" : "FORMING",
           pairing_active ? "OPEN" : "CLOSED");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");

    for (int i = 0; i < c->sensor_count; i++) {
        printf("  Sensor %d | IEEE=%s | Short=0x%04hx | Name=%s\n",
               i + 1, c->sensors[i].ieee_addr, c->sensors[i].short_addr, c->sensors[i].sensor_name);
    }

    unlock_config();
    fflush(stdout);
}

esp_err_t save_config(hub_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    nvs_handle_t handle;
    esp_err_t err = nvs_open("sensor_hub", NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    nvs_set_u8(handle, "mode", (uint8_t)config->mode);
    nvs_set_u8(handle, "sensor_count", config->sensor_count);
    nvs_set_u8(handle, "pairing_active", config->pairing_active ? 1 : 0);
    nvs_set_u8(handle, "pairing_expired", pairing_window_expired ? 1 : 0);

    for (int i = 0; i < config->sensor_count; i++) {
        char key[32];
        snprintf(key, sizeof(key), "sensor_%d", i);
        nvs_set_blob(handle, key, &config->sensors[i], sizeof(sensor_t));
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

esp_err_t load_config(hub_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    memset(g_meta, 0, sizeof(g_meta));
    pairing_window_expired = false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open("sensor_hub", NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        config->mode = MODE_PAIRING;
        config->sensor_count = 0;
        config->pairing_active = true;
        config->pairing_started = time(NULL);
        return ESP_OK;
    }
    if (err != ESP_OK) return err;

    uint8_t mode = MODE_PAIRING;
    nvs_get_u8(handle, "mode", &mode);
    config->mode = (hub_mode_t)mode;

    uint8_t pa = 1;
    nvs_get_u8(handle, "pairing_active", &pa);
    config->pairing_active = (pa != 0);

    uint8_t pe = 0;
    nvs_get_u8(handle, "pairing_expired", &pe);
    pairing_window_expired = (pe != 0);

    uint8_t sensor_count = 0;
    nvs_get_u8(handle, "sensor_count", &sensor_count);
    if (sensor_count > MAX_SENSORS) sensor_count = MAX_SENSORS;

    config->sensor_count = 0;
    for (int i = 0; i < sensor_count; i++) {
        char key[32];
        size_t size = sizeof(sensor_t);
        snprintf(key, sizeof(key), "sensor_%d", i);
        if (nvs_get_blob(handle, key, &config->sensors[i], &size) == ESP_OK) {
            config->sensor_count++;
        }
    }

    nvs_close(handle);
    RAW_LOG("[RAW] load_config complete sensor_count=%u pairing_expired=%d\n",
            (unsigned)config->sensor_count, pairing_window_expired);
    return ESP_OK;
}

static void register_or_update_joined_sensor(uint16_t short_addr, const char *ieee)
{
    hub_config_t *c = lock_config();
    if (!c) return;

    int idx = -1;
    for (int i = 0; i < c->sensor_count; i++) {
        if (strcmp(c->sensors[i].ieee_addr, ieee) == 0) {
            idx = i;
            break;
        }
    }

    if (idx < 0) {
        if (c->sensor_count >= MAX_SENSORS) {
            unlock_config();
            ESP_LOGW(TAG, "Sensor registry full");
            return;
        }
        idx = c->sensor_count++;
        memset(&c->sensors[idx], 0, sizeof(sensor_t));
        memset(&g_meta[idx], 0, sizeof(sensor_runtime_meta_t));
    }

    c->sensors[idx].short_addr = short_addr;
    strncpy(c->sensors[idx].ieee_addr, ieee, IEEE_ADDR_STR_LEN - 1);
    c->sensors[idx].ieee_addr[IEEE_ADDR_STR_LEN - 1] = '\0';
    c->sensors[idx].endpoint = 1;
    set_default_sensor_name(&c->sensors[idx], idx);

    unlock_config();
    mark_dirty();
}

static void send_fade_time_zero(uint16_t short_addr)
{
    const uint8_t payload[] = {0x00, TUYA_DP_FADING_TIME, 0x04, 0x00, 0x01, 0x00, 0x00};
    ezb_zcl_custom_cluster_cmd_t cmd_req = {0};
    uint8_t *buf = malloc(sizeof(payload));
    if (!buf) return;
    memcpy(buf, payload, sizeof(payload));

    cmd_req.cmd_ctrl.dst_addr.addr_mode = EZB_ADDR_MODE_SHORT;
    cmd_req.cmd_ctrl.dst_addr.u.short_addr = short_addr;
    cmd_req.cmd_ctrl.src_ep = COORDINATOR_ENDPOINT;
    cmd_req.cmd_ctrl.dst_ep = 1;
    cmd_req.cmd_ctrl.cluster_id = CLUSTER_PRIVATE_TUYA;
    cmd_req.cmd_ctrl.fc.direction = EZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd_req.cmd_ctrl.fc.dis_default_rsp = 1;
    cmd_req.cmd_id = 0x00;
    cmd_req.data_length = sizeof(payload);
    cmd_req.data = buf;

    (void)ezb_zcl_custom_cluster_cmd_req(&cmd_req);
    free(buf);
}

static void configure_reporting_for_model(uint16_t short_addr, uint8_t ep)
{
    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0 || g_meta[idx].reporting_configured) return;

    ezb_zcl_config_report_cmd_t cmd = {0};

    cmd.cmd_ctrl.dst_addr.addr_mode = EZB_ADDR_MODE_SHORT;
    cmd.cmd_ctrl.dst_addr.u.short_addr = short_addr;
    cmd.cmd_ctrl.src_ep = COORDINATOR_ENDPOINT;
    cmd.cmd_ctrl.dst_ep = ep;
    cmd.cmd_ctrl.fc.direction = EZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd.cmd_ctrl.fc.dis_default_rsp = 1;

    ezb_zcl_config_report_record_t batt = {
        .direction = EZB_ZCL_REPORTING_SEND,
        .attr_id = ATTR_BATTERY_PERCENT,
        .client = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT8, .min_interval = 5, .max_interval = 3600, .reportable_change = {.u8 = 1}},
    };

    cmd.cmd_ctrl.cluster_id = CLUSTER_POWER_CONFIG;
    cmd.payload.record_number = 1;
    cmd.payload.record_field = &batt;
    ESP_LOGI(TAG, "Configuring battery reporting for 0x%04hx ep %u", short_addr, ep);
    (void)ezb_zcl_config_report_cmd_req(&cmd);

    if (g_meta[idx].type == SENSOR_ZG_204ZV) {
        ezb_zcl_config_report_record_t temp = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id = ATTR_TEMPERATURE_MEASURED,
            .client = {.attr_type = EZB_ZCL_ATTR_TYPE_INT16, .min_interval = 5, .max_interval = 3600, .reportable_change = {.s16 = 10}},
        };
        cmd.cmd_ctrl.cluster_id = CLUSTER_TEMP_MEASUREMENT;
        cmd.payload.record_field = &temp;
        ESP_LOGI(TAG, "Configuring temperature reporting for 0x%04hx ep %u", short_addr, ep);
        (void)ezb_zcl_config_report_cmd_req(&cmd);

        ezb_zcl_config_report_record_t hum = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id = ATTR_HUMIDITY_MEASURED,
            .client = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT16, .min_interval = 5, .max_interval = 3600, .reportable_change = {.u16 = 100}},
        };
        cmd.cmd_ctrl.cluster_id = CLUSTER_HUMIDITY;
        cmd.payload.record_field = &hum;
        ESP_LOGI(TAG, "Configuring humidity reporting for 0x%04hx ep %u", short_addr, ep);
        (void)ezb_zcl_config_report_cmd_req(&cmd);

        ezb_zcl_config_report_record_t lux = {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id = ATTR_ILLUMINANCE_MEASURED,
            .client = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT16, .min_interval = 10, .max_interval = 3600, .reportable_change = {.u16 = 5}},
        };
        cmd.cmd_ctrl.cluster_id = CLUSTER_ILLUMINANCE;
        cmd.payload.record_field = &lux;
        ESP_LOGI(TAG, "Configuring illuminance reporting for 0x%04hx ep %u", short_addr, ep);
        (void)ezb_zcl_config_report_cmd_req(&cmd);
    }

    ezb_zcl_config_report_record_t occ = {
        .direction = EZB_ZCL_REPORTING_SEND,
        .attr_id = ATTR_OCCUPANCY,
        .client = {.attr_type = EZB_ZCL_ATTR_TYPE_UINT8, .min_interval = 1, .max_interval = 30, .reportable_change = {.u8 = 1}},
    };
    cmd.cmd_ctrl.cluster_id = CLUSTER_OCCUPANCY_SENSING;
    cmd.payload.record_field = &occ;
    ESP_LOGI(TAG, "Configuring occupancy reporting for 0x%04hx ep %u", short_addr, ep);
    (void)ezb_zcl_config_report_cmd_req(&cmd);

    g_meta[idx].reporting_configured = true;
}

static void bind_model_clusters(uint16_t short_addr, const ezb_extaddr_t *ieee, uint8_t ep)
{
    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0 || !ieee || g_meta[idx].bound_once) return;

    ezb_extaddr_t coordinator_ieee;
    ezb_get_extended_address(&coordinator_ieee);

    const uint16_t *clusters = NULL;
    size_t cluster_count = 0;

    static const uint16_t zg204_clusters[] = {CLUSTER_TEMP_MEASUREMENT, CLUSTER_POWER_CONFIG, CLUSTER_OCCUPANCY_SENSING, CLUSTER_HUMIDITY, CLUSTER_ILLUMINANCE, CLUSTER_PRIVATE_TUYA};
    static const uint16_t zg205_clusters[] = {CLUSTER_OCCUPANCY_SENSING, CLUSTER_ILLUMINANCE, CLUSTER_POWER_CONFIG, CLUSTER_PRIVATE_TUYA};
    static const uint16_t zg102_clusters[] = {CLUSTER_IAS_ZONE, CLUSTER_POWER_CONFIG};

    switch (g_meta[idx].type) {
    case SENSOR_ZG_204ZV:
        clusters = zg204_clusters;
        cluster_count = sizeof(zg204_clusters) / sizeof(zg204_clusters[0]);
        break;
    case SENSOR_ZG_205Z_A:
        clusters = zg205_clusters;
        cluster_count = sizeof(zg205_clusters) / sizeof(zg205_clusters[0]);
        break;
    case SENSOR_ZG_102Z:
    case SENSOR_ZG_102ZA:
        clusters = zg102_clusters;
        cluster_count = sizeof(zg102_clusters) / sizeof(zg102_clusters[0]);
        break;
    default:
        return;
    }

    for (size_t i = 0; i < cluster_count; i++) {
        ezb_zdo_bind_req_t *req = malloc(sizeof(ezb_zdo_bind_req_t));
        if (!req) continue;
        memset(req, 0, sizeof(*req));
        req->dst_nwk_addr = short_addr;
        req->field.src_addr = *ieee;
        req->field.src_ep = ep;
        req->field.cluster_id = clusters[i];
        req->field.dst_addr_mode = EZB_ADDR_MODE_EXT;
        req->field.dst_addr.extended_addr = coordinator_ieee;
        req->field.dst_ep = COORDINATOR_ENDPOINT;

        ESP_LOGI(TAG, "Bind request 0x%04hx ep %u cluster 0x%04hx", short_addr, ep, clusters[i]);
        ezb_zdo_bind_req(req);
        free(req);
    }

    g_meta[idx].bound_once = true;
}

static void request_model_id(uint16_t short_addr, uint8_t ep)
{
    uint16_t attrs[] = {ATTR_BASIC_MANUFACTURER_NAME, ATTR_BASIC_MODEL_IDENTIFIER};
    ezb_zcl_read_attr_cmd_t cmd = {
        .cmd_ctrl = {
            .dst_addr.addr_mode = EZB_ADDR_MODE_SHORT,
            .src_ep = COORDINATOR_ENDPOINT,
            .dst_addr.u.short_addr = short_addr,
            .dst_ep = ep,
            .cluster_id = CLUSTER_BASIC,
            .fc.direction = EZB_ZCL_CMD_DIRECTION_TO_SRV,
        },
        .payload.attr_number = 2,
        .payload.attr_field = attrs,
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t ret = ezb_zcl_read_attr_cmd_req(&cmd);
    esp_zigbee_lock_release();

    if (ret == EZB_ERR_NONE) ESP_LOGI(TAG, "Sent modelId read request to 0x%04hx ep %u", short_addr, ep);
    else ESP_LOGW(TAG, "Failed to send modelId read request to 0x%04hx ep %u: 0x%04x", short_addr, ep, ret);
}

static void zcl_core_read_attr_rsp_handler(ezb_zcl_cmd_read_attr_rsp_message_t *message)
{
    if (!message || !message->in.header || message->info.cluster_id != CLUSTER_BASIC) return;

    uint16_t short_addr = message->in.header->src_addr.u.short_addr;
    int idx = find_sensor_index_by_short(short_addr);
    if (idx < 0) return;

    ezb_zcl_read_attr_rsp_variable_t *var = message->in.variables;
    while (var) {
        if (var->status == EZB_ZCL_STATUS_SUCCESS) {
            if (var->attr_id == ATTR_BASIC_MANUFACTURER_NAME) {
                ESP_LOGI(TAG, "Manufacturer name: %.*s", *(uint8_t *)var->attr_value, (char *)(var->attr_value + 1));
            } else if (var->attr_id == ATTR_BASIC_MODEL_IDENTIFIER) {
                uint8_t len = *(uint8_t *)var->attr_value;
                char model[32] = {0};
                if (len >= sizeof(model)) len = sizeof(model) - 1;
                memcpy(model, (char *)(var->attr_value + 1), len);
                model[len] = '\0';
                apply_model_to_sensor(idx, model);
            }
        }
        var = var->next;
    }

    if (g_meta[idx].model_known) {
        ezb_extaddr_t sensor_ieee;
        if (ezb_address_extended_by_short(short_addr, &sensor_ieee) == EZB_ERR_NONE) {
            bind_model_clusters(short_addr, &sensor_ieee, message->in.header->src_ep);
            configure_reporting_for_model(short_addr, message->in.header->src_ep);
            if ((g_meta[idx].type == SENSOR_ZG_204ZV || g_meta[idx].type == SENSOR_ZG_205Z_A) && !g_meta[idx].fade_sent) {
                send_fade_time_zero(short_addr);
                g_meta[idx].fade_sent = true;
            }
        }
    }
}

static int16_t read_be16(const uint8_t *p) { return (int16_t)(((uint16_t)p[0] << 8) | p[1]); }

static void handle_zg102z_raw(uint16_t src_addr, uint16_t cluster_id, const uint8_t *payload, size_t len)
{
    (void)src_addr;
    if (cluster_id == CLUSTER_IAS_ZONE) {
        ESP_LOGI(TAG, "ZG-102Z IAS payload:");
        ESP_LOG_BUFFER_HEX(TAG, payload, len);
    }
}

static void handle_zg102za_raw(uint16_t src_addr, uint16_t cluster_id, const uint8_t *payload, size_t len)
{
    handle_zg102z_raw(src_addr, cluster_id, payload, len);
}

static void handle_zg204zv_raw(uint16_t src_addr, uint16_t cluster_id, const uint8_t *payload, size_t len)
{
    (void)src_addr;
    if (cluster_id == CLUSTER_PRIVATE_TUYA) {
        ESP_LOGI(TAG, "ZG-204ZV EF00 payload:");
        ESP_LOG_BUFFER_HEX(TAG, payload, len);
    }
}

static void handle_zg205z_a_raw(uint16_t src_addr, uint16_t cluster_id, const uint8_t *payload, size_t len)
{
    (void)src_addr;
    if (cluster_id == CLUSTER_PRIVATE_TUYA) {
        ESP_LOGI(TAG, "ZG-205Z/A EF00 payload:");
        ESP_LOG_BUFFER_HEX(TAG, payload, len);
    }
}

static void handle_model_report(uint16_t src_addr, uint16_t cluster_id, const uint8_t *payload, size_t len)
{
    int idx = find_sensor_index_by_short(src_addr);
    if (idx < 0) return;

    switch (g_meta[idx].type) {
    case SENSOR_ZG_204ZV:
        if (cluster_id == CLUSTER_TEMP_MEASUREMENT && len >= 5) {
            ESP_LOGI(TAG, "Sensor %d temperature: %.1f C", idx + 1, (double)read_be16(&payload[3]) / 100.0);
        } else if (cluster_id == CLUSTER_HUMIDITY && len >= 5) {
            ESP_LOGI(TAG, "Sensor %d humidity: %.1f %%", idx + 1, (double)read_be16(&payload[3]) / 100.0);
        } else if (cluster_id == CLUSTER_ILLUMINANCE && len >= 5) {
            ESP_LOGI(TAG, "Sensor %d illuminance raw report", idx + 1);
        } else if (cluster_id == CLUSTER_POWER_CONFIG && len >= 4 && payload[2] == 0x21) {
            ESP_LOGI(TAG, "Sensor %d battery: %u%%", idx + 1, payload[3] / 2);
        } else if (cluster_id == CLUSTER_OCCUPANCY_SENSING && len >= 1) {
            ESP_LOGI(TAG, "Sensor %d occupancy: %s", idx + 1, payload[len - 1] ? "occupied" : "vacant");
        } else if (cluster_id == CLUSTER_PRIVATE_TUYA) {
            handle_zg204zv_raw(src_addr, cluster_id, payload, len);
        }
        break;

    case SENSOR_ZG_205Z_A:
        if (cluster_id == CLUSTER_OCCUPANCY_SENSING && len >= 1) {
            ESP_LOGI(TAG, "Sensor %d occupancy: %s", idx + 1, payload[len - 1] ? "occupied" : "vacant");
        } else if (cluster_id == CLUSTER_ILLUMINANCE) {
            ESP_LOGI(TAG, "Sensor %d illuminance raw report", idx + 1);
        } else if (cluster_id == CLUSTER_POWER_CONFIG && len >= 4 && payload[2] == 0x21) {
            ESP_LOGI(TAG, "Sensor %d battery: %u%%", idx + 1, payload[3] / 2);
        } else if (cluster_id == CLUSTER_PRIVATE_TUYA) {
            handle_zg205z_a_raw(src_addr, cluster_id, payload, len);
        }
        break;

    case SENSOR_ZG_102Z:
        if (cluster_id == CLUSTER_IAS_ZONE && len >= 2) {
            uint16_t zone_status = ((uint16_t)payload[len - 2] << 8) | payload[len - 1];
            ESP_LOGI(TAG, "Sensor %d contact: %s", idx + 1, (zone_status & 0x0001) ? "open" : "closed");
            ESP_LOGI(TAG, "Sensor %d tamper: %s", idx + 1, (zone_status & 0x0004) ? "true" : "false");
            ESP_LOGI(TAG, "Sensor %d battery_low: %s", idx + 1, (zone_status & 0x0008) ? "true" : "false");
        } else if (cluster_id == CLUSTER_POWER_CONFIG && len >= 4 && payload[2] == 0x21) {
            ESP_LOGI(TAG, "Sensor %d battery: %u%%", idx + 1, payload[3] / 2);
        } else if (cluster_id == CLUSTER_IAS_ZONE) {
            handle_zg102z_raw(src_addr, cluster_id, payload, len);
        }
        break;

    case SENSOR_ZG_102ZA:
        if (cluster_id == CLUSTER_IAS_ZONE && len >= 2) {
            uint16_t zone_status = ((uint16_t)payload[len - 2] << 8) | payload[len - 1];
            ESP_LOGI(TAG, "Sensor %d contact: %s", idx + 1, (zone_status & 0x0001) ? "open" : "closed");
            ESP_LOGI(TAG, "Sensor %d tamper: %s", idx + 1, (zone_status & 0x0004) ? "true" : "false");
            ESP_LOGI(TAG, "Sensor %d battery_low: %s", idx + 1, (zone_status & 0x0008) ? "true" : "false");
        } else if (cluster_id == CLUSTER_POWER_CONFIG && len >= 4 && payload[2] == 0x21) {
            ESP_LOGI(TAG, "Sensor %d battery: %u%%", idx + 1, payload[3] / 2);
        } else if (cluster_id == CLUSTER_IAS_ZONE) {
            handle_zg102za_raw(src_addr, cluster_id, payload, len);
        }
        break;

    default:
        break;
    }
}

static void zcl_core_cmd_report_attr_handler(ezb_zcl_cmd_report_attr_message_t *message)
{
    if (!message || !message->in.header) return;
    ESP_LOGI(TAG, "Report Attribute: ep(%d) cluster(0x%04x) status(0x%02x)",
             message->info.dst_ep, message->info.cluster_id, message->info.status);
    if (!message->in.variables) return;

    int idx = find_sensor_index_by_short(message->in.header->src_addr.u.short_addr);
    if (idx < 0) return;

    switch (g_meta[idx].type) {
    case SENSOR_ZG_204ZV:
        if (message->info.cluster_id == CLUSTER_TEMP_MEASUREMENT) {
            for (ezb_zcl_report_attr_variable_t *var = message->in.variables; var; var = var->next) {
                if (var->attr_id == ATTR_TEMPERATURE_MEASURED) {
                    ESP_LOGI(TAG, "Sensor %d temperature: %.1f C", idx + 1, (double)(*(int16_t *)var->attr_value) / 100.0);
                }
            }
        } else if (message->info.cluster_id == CLUSTER_HUMIDITY) {
            for (ezb_zcl_report_attr_variable_t *var = message->in.variables; var; var = var->next) {
                if (var->attr_id == ATTR_HUMIDITY_MEASURED) {
                    ESP_LOGI(TAG, "Sensor %d humidity: %.1f %%", idx + 1, (double)(*(uint16_t *)var->attr_value) / 100.0);
                }
            }
        } else if (message->info.cluster_id == CLUSTER_POWER_CONFIG) {
            for (ezb_zcl_report_attr_variable_t *var = message->in.variables; var; var = var->next) {
                if (var->attr_id == ATTR_BATTERY_PERCENT) {
                    ESP_LOGI(TAG, "Sensor %d battery: %u%%", idx + 1, *(uint8_t *)var->attr_value);
                }
            }
        }
        break;

    case SENSOR_ZG_205Z_A:
        if (message->info.cluster_id == CLUSTER_OCCUPANCY_SENSING) {
            for (ezb_zcl_report_attr_variable_t *var = message->in.variables; var; var = var->next) {
                if (var->attr_id == ATTR_OCCUPANCY) {
                    ESP_LOGI(TAG, "Sensor %d occupancy: %s", idx + 1, (*(uint8_t *)var->attr_value) ? "occupied" : "vacant");
                }
            }
        } else if (message->info.cluster_id == CLUSTER_POWER_CONFIG) {
            for (ezb_zcl_report_attr_variable_t *var = message->in.variables; var; var = var->next) {
                if (var->attr_id == ATTR_BATTERY_PERCENT) {
                    ESP_LOGI(TAG, "Sensor %d battery: %u%%", idx + 1, *(uint8_t *)var->attr_value);
                }
            }
        }
        break;

    case SENSOR_ZG_102Z:
    case SENSOR_ZG_102ZA:
        if (message->info.cluster_id == CLUSTER_IAS_ZONE) {
            for (ezb_zcl_report_attr_variable_t *var = message->in.variables; var; var = var->next) {
                if (var->attr_id == 0x0002 || var->attr_id == 0x0000) {
                    uint16_t zone_status = *(uint16_t *)var->attr_value;
                    ESP_LOGI(TAG, "Sensor %d contact: %s", idx + 1, (zone_status & 0x0001) ? "open" : "closed");
                    ESP_LOGI(TAG, "Sensor %d tamper: %s", idx + 1, (zone_status & 0x0004) ? "true" : "false");
                }
            }
        } else if (message->info.cluster_id == CLUSTER_POWER_CONFIG) {
            for (ezb_zcl_report_attr_variable_t *var = message->in.variables; var; var = var->next) {
                if (var->attr_id == ATTR_BATTERY_PERCENT) {
                    ESP_LOGI(TAG, "Sensor %d battery: %u%%", idx + 1, *(uint8_t *)var->attr_value);
                }
            }
        }
        break;

    default:
        break;
    }
}

static void zcl_core_cmd_report_config_rsp_handler(ezb_zcl_cmd_config_report_rsp_message_t *message)
{
    if (!message) return;
    ESP_LOGI(TAG, "Configure Reporting Response: ep(%d) cluster(0x%04x) status(0x%02x)",
             message->info.dst_ep, message->info.cluster_id, message->info.status);
}

static void zcl_core_cmd_default_rsp_handler(ezb_zcl_cmd_default_rsp_message_t *message)
{
    if (!message) return;
    ESP_LOGI(TAG, "Received ZCL Default Response: 0x%02x", message->in.status_code);
}

static void esp_zigbee_zcl_core_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {
    case EZB_ZCL_CORE_READ_ATTR_RSP_CB_ID:
        zcl_core_read_attr_rsp_handler((ezb_zcl_cmd_read_attr_rsp_message_t *)message);
        break;
    case EZB_ZCL_CORE_CONFIG_REPORT_RSP_CB_ID:
        zcl_core_cmd_report_config_rsp_handler((ezb_zcl_cmd_config_report_rsp_message_t *)message);
        break;
    case EZB_ZCL_CORE_REPORT_ATTR_CB_ID:
        zcl_core_cmd_report_attr_handler((ezb_zcl_cmd_report_attr_message_t *)message);
        break;
    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID:
        zcl_core_cmd_default_rsp_handler((ezb_zcl_cmd_default_rsp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "ZCL Core Action: ID(0x%04lx)", callback_id);
        break;
    }
}

static void deferred_formation_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    if (!formation_requested) {
        formation_requested = true;
        RAW_LOG("[RAW] starting network formation\n");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION);
    }
    vTaskDelete(NULL);
}

static void pairing_window_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(PAIRING_DURATION_SEC * 1000));
    pairing_window_expired = true;
    pairing_active = false;
    ezb_bdb_open_network(0);
    print_sensor_summary();
    vTaskDelete(NULL);
}

static void persist_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (g_dirty) {
            hub_config_t *c = lock_config();
            if (c) {
                save_config(c);
                unlock_config();
                g_dirty = false;
            }
        }
    }
}

static void active_ep_callback(const ezb_zdo_active_ep_req_result_t *result, void *user_ctx)
{
    uint16_t short_addr = (uint16_t)(uintptr_t)user_ctx;
    if (!result || result->error != EZB_ERR_NONE || !result->rsp) return;
    for (uint8_t i = 0; i < result->rsp->active_ep_count; i++) request_model_id(short_addr, result->rsp->active_ep_list[i]);
}

static bool raw_frame_handler(const ezb_zcl_raw_frame_t *raw_frame)
{
    if (!raw_frame || !raw_frame->header) return false;

    uint16_t src_addr = raw_frame->header->src_addr.u.short_addr;
    uint16_t cluster_id = raw_frame->header->cluster_id;
    RAW_LOG("[RAW] frame src=0x%04hx cluster=0x%04hx plen=%u\n", src_addr, cluster_id, (unsigned)raw_frame->payload_length);

    if (cluster_id == CLUSTER_PRIVATE_TUYA) {
        RAW_LOG("[RAW] EF00 bytes:");
        for (uint16_t i = 0; i < raw_frame->payload_length; i++) RAW_LOG(" %02X", raw_frame->payload[i]);
        RAW_LOG("\n");
    }

    handle_model_report(src_addr, cluster_id, raw_frame->payload, raw_frame->payload_length);
    return false;
}

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    switch (ezb_app_signal_get_type(app_signal)) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        if (!formation_task_started) {
            formation_task_started = true;
            xTaskCreate(deferred_formation_task, "zb_form", 3072, NULL, 5, NULL);
        }
        break;

    case EZB_BDB_SIGNAL_FORMATION: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            network_formed = true;
            zigbee_ready = true;
            pairing_window_expired = false;
            pairing_active = true;
            ezb_bdb_open_network((uint8_t)PAIRING_DURATION_SEC);
            xTaskCreate(pairing_window_task, "pair_win", 3072, NULL, 4, NULL);
        } else {
            formation_requested = false;
        }
        break;
    }

    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        const ezb_zdo_signal_device_annce_params_t *ann = ezb_app_signal_get_params(app_signal);
        if (!ann) break;

        char ieee_str[IEEE_ADDR_STR_LEN] = {0};
        snprintf(ieee_str, sizeof(ieee_str),
                 "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 ann->device_addr.u8[7], ann->device_addr.u8[6], ann->device_addr.u8[5], ann->device_addr.u8[4],
                 ann->device_addr.u8[3], ann->device_addr.u8[2], ann->device_addr.u8[1], ann->device_addr.u8[0]);

        register_or_update_joined_sensor(ann->short_addr, ieee_str);

        ezb_zdo_active_ep_req_t req = {0};
        req.dst_nwk_addr = ann->short_addr;
        req.field.nwk_addr_of_interest = ann->short_addr;
        req.cb = active_ep_callback;
        req.user_ctx = (void *)(uintptr_t)ann->short_addr;
        ezb_zdo_active_ep_req(&req);
        break;
    }

    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t duration = *(uint8_t *)ezb_app_signal_get_params(app_signal);
        pairing_active = (duration != 0);
        if (duration == 0 && !pairing_window_expired) {
            pairing_window_expired = true;
            print_sensor_summary();
        }
        break;
    }

    default:
        break;
    }
    return true;
}

static esp_err_t esp_zigbee_create_coordinator_device(void)
{
    static const char manufacturer_name[] = "Innovatsii EMS";
    static const char model_identifier[] = "sensor-hub-zigbee";

    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();
    ezb_zha_custom_gateway_config_t gateway_cfg = EZB_ZHA_CUSTOM_GATEWAY_CONFIG();
    ezb_af_ep_desc_t ep_desc = ezb_zha_create_custom_gateway(COORDINATOR_ENDPOINT, &gateway_cfg);

    ezb_zcl_cluster_desc_t basic_desc = ezb_af_endpoint_get_cluster_desc(ep_desc, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)manufacturer_name);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)model_identifier);

    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc, ezb_zcl_on_off_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER)));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc, ezb_zcl_occupancy_sensing_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER)));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));

    ezb_zcl_raw_command_handler_register(raw_frame_handler);
    ezb_zcl_core_action_handler_register(esp_zigbee_zcl_core_action_handler);
    return ESP_OK;
}

static esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));
    return ESP_OK;
}

static void esp_zigbee_stack_main_task(void *pvParameters)
{
    (void)pvParameters;
    esp_zigbee_config_t zigbee_config = {0};
    zigbee_config.device_config.device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR;
    zigbee_config.device_config.install_code_policy = false;
    zigbee_config.device_config.zczr_config.max_children = MAX_SENSORS;
    zigbee_config.platform_config.storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME;
    zigbee_config.platform_config.radio_config.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE;

    ESP_ERROR_CHECK(esp_zigbee_init(&zigbee_config));
    ESP_ERROR_CHECK(esp_zigbee_setup_commissioning());
    ESP_ERROR_CHECK(esp_zigbee_create_coordinator_device());
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    ESP_ERROR_CHECK(esp_zigbee_launch_mainloop());

    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

void app_main(void)
{
    print_banner();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME));

    if (FACTORY_RESET_MODE) {
        // factory reset would go here if enabled
    }

    g_config.mutex = xSemaphoreCreateMutex();
    if (!g_config.mutex) {
        ESP_LOGE(TAG, "Mutex creation failed");
        return;
    }

    memset(&g_config.data, 0, sizeof(g_config.data));
    g_config.data.mode = MODE_PAIRING;
    g_config.data.pairing_active = true;
    g_config.data.pairing_started = time(NULL);

    load_config(&g_config.data);

    xTaskCreate(esp_zigbee_stack_main_task, "Zigbee_main", 4096 * 2, NULL, 5, NULL);
    xTaskCreate(persist_task, "persist_task", 2048, NULL, 3, NULL);
}