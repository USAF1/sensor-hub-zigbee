/*
 * main.h — Sensor Hub Zigbee Coordinator
 * Innovatsii EMS — Pico 1
 * Firmware Version: 0.2.5
 */

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ezbee/core_types.h"

// ============================================================================
// FIRMWARE VERSION
// ============================================================================

#define FIRMWARE_VERSION    "0.2.5"
#define FIRMWARE_COMPONENT  "sensor_hub"

// ============================================================================
// CONSTANTS
// ============================================================================

#define MAX_SENSORS          15
#define POLL_INTERVAL_MS     5000
#define PAIRING_TIMEOUT_SEC  120
#define IEEE_ADDR_STR_LEN    24
#define SENSOR_NAME_LEN      32
#define COORDINATOR_ENDPOINT 1

#define WATCHDOG_OFFLINE_PAIRING_THRESHOLD  3
#define WATCHDOG_PAIRING_REOPEN_SEC         30

// ============================================================================
// ENUMS
// ============================================================================

typedef enum {
    MODE_PAIRING = 0,
    MODE_NORMAL  = 1,
} hub_mode_t;

typedef enum {
    ROLE_UNASSIGNED = 0,
    ROLE_DOOR       = 1,
    ROLE_PRESENCE   = 2,
} sensor_role_t;

typedef enum {
    UNIT_VACANT   = 0,
    UNIT_OCCUPIED = 1,
} unit_occupancy_t;

typedef enum {
    SENSOR_UNKNOWN   = 0,
    SENSOR_ZG_204ZV  = 1,
    SENSOR_ZG_205Z_A = 2,
    SENSOR_ZG_102Z   = 3,
    SENSOR_ZG_102ZA  = 4,
} sensor_type_t;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

typedef struct {
    char     ieee_addr[IEEE_ADDR_STR_LEN];
    uint16_t short_addr;
    uint8_t  endpoint;
    char     sensor_name[SENSOR_NAME_LEN];
    uint8_t  sensor_type;
    uint8_t  sensor_role;
    bool     online;
    uint8_t  ping_attempts;
    bool     presence;
    bool     contact_open;
    bool     tamper;
    bool     battery_low;
    int16_t  temperature_cdeg;
    uint16_t humidity_cpct;
    uint8_t  battery_pct;
    time_t   last_seen;
    time_t   last_change;
} sensor_t;

typedef struct {
    bool   occupied;
    time_t timestamp;
    time_t last_change;
} hub_status_t;

typedef struct {
    unit_occupancy_t unit_state;
    time_t           unit_state_changed;
    bool             door_closed_pending;
    time_t           door_closed_at;
    hub_status_t     hub_status;
    sensor_t         sensors[MAX_SENSORS];
    uint8_t          sensor_count;
    hub_mode_t       mode;
    bool             pairing_active;
    time_t           pairing_started;
} hub_config_t;

typedef struct {
    hub_config_t      data;
    SemaphoreHandle_t mutex;
} hub_config_safe_t;

typedef struct {
    bool    model_known;
    char    model_id[32];
    bool    bound_once;
    bool    reporting_configured;
    bool    enroll_sent;
    bool    fade_sent;
    bool    ping_pending;
    uint8_t rejoin_count;
    uint8_t miss_count;
    uint8_t ep_pending;
} sensor_runtime_meta_t;

// ============================================================================
// GLOBALS
// ============================================================================

extern hub_config_safe_t      g_config;
extern sensor_runtime_meta_t  g_meta[MAX_SENSORS];
extern volatile bool          g_watchdog_started;
extern volatile int           g_new_sensor_count;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

hub_config_t *lock_config(void);
void          unlock_config(void);
esp_err_t     save_config(hub_config_t *config);
esp_err_t     load_config(hub_config_t *config);
void          mark_dirty(void);
const char   *friendly_name_from_type(sensor_type_t t);
const char   *unit_state_str(unit_occupancy_t s);
const char   *role_str(sensor_role_t r);

#endif /* MAIN_H */