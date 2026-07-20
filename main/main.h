#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ============================================================================
// CONSTANTS
// ============================================================================

#define MAX_SENSORS           12
#define POLL_INTERVAL_MS      5000
#define PAIRING_TIMEOUT_SEC   120
#define IEEE_ADDR_STR_LEN     24
#define SENSOR_NAME_LEN       32
#define COORDINATOR_ENDPOINT  1

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Operating modes
 */
typedef enum {
    MODE_PAIRING = 0,
    MODE_NORMAL = 1,
} hub_mode_t;

/**
 * @brief Sensor data structure (runtime)
 */
typedef struct {
    char ieee_addr[IEEE_ADDR_STR_LEN];
    uint16_t short_addr;
    char sensor_name[SENSOR_NAME_LEN];
    uint8_t status;              // 0 = Vacant, 1 = Occupied
    uint8_t endpoint;            // Zigbee endpoint
    time_t last_polled;
    time_t last_response;
    uint32_t poll_count;
    uint32_t failed_polls;
    int8_t rssi;                 // Signal strength (dBm)
} sensor_t;

/**
 * @brief Hub status aggregation
 */
typedef struct {
    uint8_t overall_status;      // 0 = Vacant, 1 = Occupied
    time_t timestamp;
    time_t last_change;
} hub_status_t;

/**
 * @brief Hub configuration
 */
typedef struct {
    hub_status_t hub_status;
    sensor_t sensors[MAX_SENSORS];
    uint8_t sensor_count;
    hub_mode_t mode;
    bool pairing_active;
    time_t pairing_started;
} hub_config_t;

/**
 * @brief Thread-safe config wrapper
 */
typedef struct {
    hub_config_t data;
    SemaphoreHandle_t mutex;
} hub_config_safe_t;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Thread safety
hub_config_t* lock_config(void);
void unlock_config(void);

// Sensor management
esp_err_t sensor_add(hub_config_t *config, const char *ieee_addr, uint16_t short_addr, uint8_t endpoint);
esp_err_t sensor_update_status(hub_config_t *config, const char *ieee_addr, uint8_t status, int8_t rssi);
sensor_t* sensor_find_by_short_addr(hub_config_t *config, uint16_t short_addr);
uint8_t sensor_aggregate_status(hub_config_t *config);
void print_sensors(hub_config_t *config);

// NVS persistence
esp_err_t save_config(hub_config_t *config);
esp_err_t load_config(hub_config_t *config);

#endif // MAIN_H