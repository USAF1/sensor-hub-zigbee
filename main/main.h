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

#define MAX_SENSORS          15
#define POLL_INTERVAL_MS     5000
#define PAIRING_TIMEOUT_SEC  120
#define IEEE_ADDR_STR_LEN    24
#define SENSOR_NAME_LEN      32
#define COORDINATOR_ENDPOINT 1

// ============================================================================
// DATA STRUCTURES
// ============================================================================

typedef enum {
    MODE_PAIRING = 0,
    MODE_NORMAL  = 1,
} hub_mode_t;

/**
 * @brief Per-sensor runtime + persisted data.
 *
 * ZG-204ZV          : presence (IAS), temp, humidity, illuminance, battery
 * ZG-205Z/A         : presence (IAS + occupancy), illuminance
 * ZG-102Z / ZG-102ZA: contact (IAS), tamper, battery_low, battery
 */
typedef struct {
    /* Identity */
    char     ieee_addr[IEEE_ADDR_STR_LEN];
    uint16_t short_addr;
    uint8_t  endpoint;
    char     sensor_name[SENSOR_NAME_LEN];
    uint8_t  sensor_type;       /* sensor_type_t stored as uint8 for NVS */

    /* Health */
    bool     online;            /* true = actively communicating          */
    uint8_t  ping_attempts;     /* watchdog retry counter                 */

    /* Presence / contact */
    bool     presence;          /* ZG-204ZV, ZG-205Z/A: true = occupied   */
    bool     contact_open;      /* ZG-102Z/A: true = door open            */
    bool     tamper;
    bool     battery_low;

    /* Environmental — ZG-204ZV only */
    int16_t  temperature_cdeg;  /* deg C * 100,  e.g. 2150 = 21.50 C     */
    uint16_t humidity_cpct;     /* % * 100,      e.g. 5000 = 50.00 %     */
    uint16_t illuminance_raw;   /* ZCL lux value                          */

    /* Battery — ZG-204ZV and ZG-102Z/A */
    uint8_t  battery_pct;       /* 0-100 %                                */

    /* Timestamps */
    time_t   last_seen;
    time_t   last_change;
} sensor_t;

typedef struct {
    bool   occupied;
    time_t timestamp;
    time_t last_change;
} hub_status_t;

typedef struct {
    hub_status_t hub_status;
    sensor_t     sensors[MAX_SENSORS];
    uint8_t      sensor_count;
    hub_mode_t   mode;
    bool         pairing_active;
    time_t       pairing_started;
} hub_config_t;

typedef struct {
    hub_config_t      data;
    SemaphoreHandle_t mutex;
} hub_config_safe_t;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

hub_config_t *lock_config(void);
void          unlock_config(void);

esp_err_t save_config(hub_config_t *config);
esp_err_t load_config(hub_config_t *config);

#endif /* MAIN_H */