/*
 * uart_hooks.c
 * Bridge between uart_master.c commands and main.c / Zigbee stack.
 *
 * uart_master.c → uart_hooks.c → main.c / Zigbee stack
 *
 * These functions are called by uart_master.c command handlers.
 * Keeping them here means uart_master.c has zero dependency on main.c.
 *
 * remove_sensor: also resets g_meta[idx].bound_once so the sensor
 * will be fully re-bound if it re-pairs. This is the ONLY place
 * bound_once is reset — never on re-announce, never on reboot.
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

    /* Shift remaining sensors down */
    for (int i = idx; i < c->sensor_count - 1; i++)
        c->sensors[i] = c->sensors[i + 1];
    memset(&c->sensors[c->sensor_count - 1], 0, sizeof(sensor_t));
    c->sensor_count--;

    /*
     * Shift g_meta array to match.
     * Reset bound_once for removed slot — if sensor re-pairs it gets
     * a full fresh binding. This is the ONLY place bound_once is reset.
     */
    for (int i = idx; i < c->sensor_count; i++)
        g_meta[i] = g_meta[i + 1];
    memset(&g_meta[c->sensor_count], 0, sizeof(sensor_runtime_meta_t));

    int remaining = c->sensor_count;
    unlock_config();

    mark_dirty();
    ESP_LOGI(TAG, "Sensor %d removed — %d remaining", idx, remaining);
}

void uart_cmd_factory_reset(void)
{
    ESP_LOGW(TAG, "FACTORY RESET — erasing NVS and restarting");
    nvs_flash_erase();
    esp_restart();
}