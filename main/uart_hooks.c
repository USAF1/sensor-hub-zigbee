/*
 * uart_hooks.c
 * Bridge between uart_master.c commands and main.c / Zigbee stack.
 *
 *   uart_master.c → uart_hooks.c → main.c / Zigbee stack
 *
 * remove_sensor shifts g_meta[] and clears the freed slot so a re-paired
 * sensor gets a fresh binding. This is the ONLY place bound_once is reset —
 * never on re-announce, never on reboot.
 *
 * set_sensor_config forwards fading_time / motion sensitivity to a presence
 * sensor via Tuya EF00 datapoint writes (DP 102 / DP 2), exactly like Z2M.
 */

#include "uart_master.h"
#include "main.h"

#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_zigbee.h"
#include "ezbee/bdb.h"

static const char *TAG = "UART_HOOKS";

void uart_cmd_start_pairing(uint16_t duration_sec)
{
    if (duration_sec == 0 || duration_sec > 600) duration_sec = 120;
    uint8_t dur8 = (duration_sec > 255) ? 255 : (uint8_t)duration_sec;
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_bdb_open_network(dur8);
    esp_zigbee_lock_release();
    ESP_LOGI(TAG, "Pairing opened for %us", (unsigned)duration_sec);
}

void uart_cmd_stop_pairing(void)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_bdb_open_network(0);
    esp_zigbee_lock_release();
    ESP_LOGI(TAG, "Pairing closed");
}

void uart_cmd_remove_sensor(int idx)
{
    if (idx < 0 || idx >= MAX_SENSORS) return;

    hub_config_t *c = lock_config();
    if (!c) return;

    if (idx >= c->sensor_count) {
        unlock_config();
        ESP_LOGW(TAG, "remove_sensor: index %d out of range (count=%d)",
                 idx, c->sensor_count);
        return;
    }

    /* Shift sensors down, then g_meta down, then clear the freed slot. */
    for (int i = idx; i < c->sensor_count - 1; i++)
        c->sensors[i] = c->sensors[i + 1];
    memset(&c->sensors[c->sensor_count - 1], 0, sizeof(sensor_t));

    for (int i = idx; i < c->sensor_count - 1; i++)
        g_meta[i] = g_meta[i + 1];
    memset(&g_meta[c->sensor_count - 1], 0, sizeof(sensor_runtime_meta_t));

    c->sensor_count--;
    int remaining = c->sensor_count;
    unlock_config();

    mark_dirty();
    ESP_LOGI(TAG, "Sensor %d removed — %d remaining", idx, remaining);
}

/*
 * uart_cmd_set_sensor_config — apply fading_time and/or motion sensitivity
 * to a presence sensor. Pass -1 for any field to leave it unchanged.
 *   fading_sec  : 0..28800   (motion keep time, seconds)
 *   sensitivity : 0..19      (motion detection sensitivity)
 * main.c validates ranges, persists in g_meta[], and sends the Tuya DP write.
 */
void uart_cmd_set_sensor_config(int idx, int fading_sec, int sensitivity)
{
    if (idx < 0 || idx >= MAX_SENSORS) return;

    hub_config_t *c = lock_config();
    if (!c) return;
    if (idx >= c->sensor_count) {
        unlock_config();
        ESP_LOGW(TAG, "set_sensor_config: index %d out of range (count=%d)",
                 idx, c->sensor_count);
        return;
    }
    unlock_config();

    ESP_LOGI(TAG, "set_sensor_config idx=%d fading=%d sensitivity=%d",
             idx, fading_sec, sensitivity);
    hub_set_sensor_config(idx, fading_sec, sensitivity);
}

void uart_cmd_factory_reset(void)
{
    ESP_LOGW(TAG, "FACTORY RESET — erasing NVS and restarting");
    nvs_flash_erase();
    esp_restart();
}