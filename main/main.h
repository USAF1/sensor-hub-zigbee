/*
 * main.h — Sensor Hub Zigbee Coordinator
 * Innovatsii EMS — Pico 1
 * Firmware Version: 0.2.5
 *
 * V4.2+ Architecture — Hub is pure data reporter:
 *
 *   Hub responsibilities:
 *     - Sensor identification (ZG-204ZV, ZG-205Z/A, ZG-102Z)
 *     - Raw sensor event reporting (presence, door, battery, environment)
 *     - Hub aggregate = OR of all online presence sensors (no door logic)
 *     - Real UTC timestamps on all outbound messages
 *     - Watchdog for always-on sensors
 *     - Never calculates unit occupancy
 *
 *   Master responsibilities (moved from Hub):
 *     - Unit occupancy calculation from door + presence events
 *     - Buffer period
 *     - Booking window + 4-state decision
 *     - Relay control
 *
 *   UTC timestamp:
 *     Hub receives utc_epoch in hub_init.
 *     All outbound messages use: utc_epoch + uptime_seconds
 *
 *   hub_aggregate:
 *     Replaces unit_occupancy. Simple OR of online presence sensors.
 *     No door sensor involvement.
 *     Sent to Master on every change.
 *
 *   Sensor joining logic: UNCHANGED from working version.
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
#define IEEE_ADDR_STR_LEN    24
#define SENSOR_NAME_LEN      32
#define COORDINATOR_ENDPOINT 1

/* Watchdog */
#define WATCHDOG_OFFLINE_PAIRING_THRESHOLD  3
#define WATCHDOG_PAIRING_REOPEN_SEC         30

/* Rejoin */
#define REJOIN_RETRY_COUNT    6
#define REJOIN_RETRY_DELAY_MS 10000
#define REJOIN_POLL_GAP_MS    500

/* ZG-204ZV fading time default — 0 means no hold (sensor goes NO immediately) */
#define PRESENCE_FADING_TIME_DEFAULT_SEC 0

/* Model ID read timeout — ZG-102Z (sleepy) never responds → infer door type */
#define MODEL_ID_TIMEOUT_MS  5000

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
    HUB_AGG_VACANT   = 0,
    HUB_AGG_OCCUPIED = 1,
} hub_aggregate_t;

typedef enum {
    SENSOR_UNKNOWN   = 0,
    SENSOR_ZG_204ZV  = 1,
    SENSOR_ZG_205Z_A = 2,
    SENSOR_ZG_102Z   = 3,
    SENSOR_ZG_102ZA  = 4,
} sensor_type_t;

typedef enum {
    ID_STATE_UNIDENTIFIED  = 0,
    ID_STATE_EP_REQUESTED  = 1,
    ID_STATE_BINDING       = 2,
    ID_STATE_MODEL_READ    = 3,
    ID_STATE_IDENTIFIED    = 4,
    ID_STATE_OPERATIONAL   = 5,
} sensor_id_state_t;

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
    time_t   door_opened_at;   /* epoch when door last went OPEN — for Master */
} sensor_t;

typedef struct {
    hub_aggregate_t  aggregate;    /* OR of all online presence sensors */
    time_t           timestamp;
    time_t           last_change;
} hub_status_t;

typedef struct {
    /* Hub aggregate — simple presence OR, no door logic */
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
    sensor_id_state_t id_state;
    char              model_id[32];
    bool              model_known;

    uint8_t           bind_pending;
    uint8_t           bind_confirmed;
    uint8_t           bind_failed;
    bool              bound_once;
    bool              power_config_bound;
    uint8_t           ep_active;

    bool              reporting_configured;
    bool              fade_sent;

    uint32_t          model_id_req_ms;
    bool              model_id_pending;

    bool              ping_pending;
    uint8_t           miss_count;
    uint8_t           rejoin_count;

    bool              enroll_sent;
} sensor_runtime_meta_t;

// ============================================================================
// GLOBALS
// ============================================================================

extern hub_config_safe_t      g_config;
extern sensor_runtime_meta_t  g_meta[MAX_SENSORS];
extern volatile bool          g_watchdog_started;
extern volatile int           g_new_sensor_count;

/* UTC epoch received from Master in hub_init — used for real timestamps */
extern volatile int64_t       g_utc_boot_epoch;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

hub_config_t *lock_config(void);
void          unlock_config(void);
esp_err_t     save_config(hub_config_t *config);
esp_err_t     load_config(hub_config_t *config);
void          mark_dirty(void);
const char   *friendly_name_from_type(sensor_type_t t);
const char   *hub_aggregate_str(hub_aggregate_t a);
const char   *role_str(sensor_role_t r);

#endif /* MAIN_H */