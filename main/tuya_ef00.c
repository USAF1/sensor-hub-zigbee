/*
 * tuya_ef00.c — Tuya EF00 cluster write helpers
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 *
 * Each write spawns a tiny one-shot FreeRTOS task.  The arg struct is
 * heap-allocated (bounded: sizeof(tuya_write_t) < 16 bytes) and freed
 * by the task before it self-deletes.  If xTaskCreate fails the struct
 * is freed immediately — no leak in any code path.
 *
 * CRITICAL: do not move the arg allocation onto a task stack.
 */

#include "tuya_ef00.h"

#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_zigbee.h"
#include "ezbee/zcl/zcl_general_cmd.h"

#include "main.h"
#include "sensor_driver.h"

static const char *TAG = "TUYA_EF00";

#define CLUSTER_PRIVATE_TUYA 0xEF00
#define TUYA_CMD_DATA_REQUEST 0x00
#define TUYA_TYPE_VALUE       0x02
#define TUYA_TYPE_ENUM        0x04

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

typedef struct {
    uint16_t sa;
    uint8_t  ep;
    uint8_t  dp;
    uint8_t  dtype;   /* TUYA_TYPE_VALUE or TUYA_TYPE_ENUM */
    uint8_t  seq;     /* Tuya sequence number, assigned before task creation */
    uint32_t val;
} tuya_write_t;

static uint8_t s_tuya_seq = 0;

static void tuya_write_task(void *arg)
{
    tuya_write_t *w = (tuya_write_t *)arg;
    uint8_t seq = w->seq;

    uint8_t payload[12];
    uint8_t data_len;

    if (w->dtype == TUYA_TYPE_ENUM) {
        /* ENUM: 1-byte value */
        payload[0] = w->dp;
        payload[1] = TUYA_TYPE_ENUM;
        payload[2] = 0x00;
        payload[3] = 0x01;
        payload[4] = (uint8_t)(w->val & 0xFF);
        data_len = 5;
    } else {
        /* VALUE: 4-byte big-endian uint32 */
        payload[0] = w->dp;
        payload[1] = TUYA_TYPE_VALUE;
        payload[2] = 0x00;
        payload[3] = 0x04;
        payload[4] = (uint8_t)((w->val >> 24) & 0xFF);
        payload[5] = (uint8_t)((w->val >> 16) & 0xFF);
        payload[6] = (uint8_t)((w->val >>  8) & 0xFF);
        payload[7] = (uint8_t)((w->val      ) & 0xFF);
        data_len = 8;
    }

    /* Prepend the Tuya 4-byte header in a full frame for logging, but the
     * ZCL custom-cluster API takes only the payload after the header. */
    (void)seq;

    ezb_zcl_custom_cluster_cmd_t cmd = {0};
    cmd.cmd_ctrl.dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT;
    cmd.cmd_ctrl.dst_addr.u.short_addr = w->sa;
    cmd.cmd_ctrl.src_ep                = COORDINATOR_ENDPOINT;
    cmd.cmd_ctrl.dst_ep                = w->ep;
    cmd.cmd_ctrl.cluster_id            = CLUSTER_PRIVATE_TUYA;
    cmd.cmd_ctrl.fc.direction          = EZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd.cmd_ctrl.fc.dis_default_rsp    = 1;
    cmd.cmd_id                         = TUYA_CMD_DATA_REQUEST;
    cmd.data_length                    = data_len;
    cmd.data                           = payload;

    esp_zigbee_lock_acquire(portMAX_DELAY);
    (void)ezb_zcl_custom_cluster_cmd_req(&cmd);
    esp_zigbee_lock_release();

    PROD_LOG(TAG, "Tuya write DP%u dtype=0x%02x val=%lu → 0x%04hx",
             w->dp, w->dtype, (unsigned long)w->val, w->sa);
    free(w);
    vTaskDelete(NULL);
}

static void tuya_write_internal(uint16_t sa, uint8_t ep, uint8_t dp,
                                 uint8_t dtype, uint32_t val)
{
    tuya_write_t *w = malloc(sizeof(tuya_write_t));
    if (!w) { ESP_LOGW(TAG, "malloc failed for Tuya write DP%u", dp); return; }
    w->sa    = sa;
    w->ep    = ep;
    w->dp    = dp;
    w->dtype = dtype;
    w->val   = val;
    w->seq   = s_tuya_seq++;   /* assign before spawning — single-threaded caller */
    if (xTaskCreate(tuya_write_task, "tuyawr", 3072, w, 3, NULL) != pdPASS) {
        ESP_LOGW(TAG, "xTaskCreate failed for Tuya write DP%u", dp);
        free(w);
    }
}

void tuya_write_dp(uint16_t sa, uint8_t ep, uint8_t dp, uint32_t val)
{
    tuya_write_internal(sa, ep, dp, TUYA_TYPE_VALUE, val);
}

void tuya_write_dp_enum(uint16_t sa, uint8_t ep, uint8_t dp, uint8_t val)
{
    tuya_write_internal(sa, ep, dp, TUYA_TYPE_ENUM, (uint32_t)val);
}

/* ── hub_set_sensor_config ────────────────────────────────────────────────── */

void hub_set_sensor_config(int idx, int keep_time_sec, int sensitivity)
{
    if (idx < 0 || idx >= MAX_SENSORS) return;
    hub_config_t *c = lock_config();
    if (!c || idx >= c->sensor_count) { if (c) unlock_config(); return; }
    sensor_type_t t = (sensor_type_t)c->sensors[idx].sensor_type;
    unlock_config();

    const sensor_driver_t *drv = sensor_driver_find(t);
    if (!drv || !drv->apply_config) {
        ESP_LOGW(TAG, "hub_set_sensor_config: idx %d has no apply_config", idx);
        return;
    }
    drv->apply_config(idx, keep_time_sec, sensitivity);
}
