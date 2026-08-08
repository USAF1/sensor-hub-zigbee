/*
 * main.c -- Sensor Hub Zigbee Coordinator -- Entry Point
 * Innovatsii EMS -- Pico 1  |  Firmware 0.3.0
 *
 * app_main is intentionally slim: NVS init, config mutex, load config,
 * UART init, and task creation.  All substantive logic lives in:
 *   zigbee_core.c    -- Zigbee stack, commissioning, signals, watchdog
 *   sensor_registry.c -- Sensor registry, NVS persistence
 *   sensor_identify.c -- EP request, binding, model-ID, driver dispatch
 *   zcl_dispatch.c   -- ZCL/Tuya callback routing
 *   hub_aggregate.c  -- Presence OR aggregate
 *   tuya_ef00.c      -- Tuya EF00 write path
 *   devices/         -- Per-device drivers (pir_zg204zl, door_zg102z)
 *
 * FACTORY_RESET_MODE:
 *   0 = normal (persist app NVS + Zigbee network across reboots)  <- production
 *   1 = wipe BOTH the app NVS AND the zb_storage partition on boot
 *       Use once to recover from a stale/mismatched zb_storage.
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "main.h"
#include "uart_master.h"
#include "sensor_registry.h"
#include "sensor_identify.h"
#include "zigbee_core.h"

#ifndef ESP_ZIGBEE_STORAGE_PARTITION_NAME
#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "zb_storage"
#endif

#define TAG "SENSOR_HUB"

/*
 * PRODUCTION VALUE: 0.  Set to 1 only for a single boot to recover from a
 * stale Zigbee network store (e.g. flash chip swapped, PAN changed).
 */
#define FACTORY_RESET_MODE  0

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

void app_main(void)
{
    printf("\nINNOVATSII EMS - SENSOR HUB v%s\n", FIRMWARE_VERSION);
    fflush(stdout);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME));

#if FACTORY_RESET_MODE
    ESP_LOGW(TAG, "FACTORY RESET -- erasing app NVS AND zb_storage");
    nvs_flash_deinit_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME);
    nvs_flash_erase_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME);
    nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME);
    nvs_flash_erase();
    nvs_flash_init();
#endif

    g_config.mutex = xSemaphoreCreateMutex();
    if (!g_config.mutex) { ESP_LOGE(TAG, "mutex failed"); return; }

    memset(&g_config.data, 0, sizeof(g_config.data));
    load_config(&g_config.data);

    esp_err_t ue = uart_master_init();
    if (ue != ESP_OK)
        ESP_LOGW(TAG, "UART init failed (%s)", esp_err_to_name(ue));

    PROD_LOG(TAG, "v%s ready -- waiting for Master ping", FIRMWARE_VERSION);

    xTaskCreate(zigbee_main_task,      "Zigbee_main", 8192, NULL, 5, NULL);
    xTaskCreate(persist_task,          "persist",     2048, NULL, 3, NULL);
    xTaskCreate(model_id_timeout_task, "id_timeout",  2048, NULL, 2, NULL);
}
