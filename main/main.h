/*
 * main.h — Sensor Hub Zigbee Coordinator
 * Innovatsii EMS — Pico 1
 *
 * Phase 2 changes vs Phase 1:
 *   - Added sensor_role_t enum (ROLE_DOOR / ROLE_PRESENCE)
 *   - Added unit_occupancy_t enum (UNIT_VACANT / UNIT_OCCUPIED)
 *   - Added sensor_role field to sensor_t
 *   - Added online field to sensor_t (watchdog health tracking)
 *   - Added ping_attempts field to sensor_t
 *   - Removed illuminance_raw from sensor_t (not parsed — too many updates)
 *   - Added unit_state and door_closed_pending to hub_config_t
 *   - Added friendly_name_from_type() declaration (used by uart_master.c)
 *   - Added mark_dirty() declaration (used by uart_hooks.c)
 */

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
// ENUMS
// ============================================================================

typedef enum {
    MODE_PAIRING = 0,
    MODE_NORMAL  = 1,
} hub_mode_t;

typedef enum {
    ROLE_UNASSIGNED = 0,
    ROLE_DOOR       = 1,   /* ZG-102Z / ZG-102ZA — main entry door   */
    ROLE_PRESENCE   = 2,   /* ZG-204ZV / ZG-205Z/A — room presence   */
} sensor_role_t;

typedef enum {
    UNIT_VACANT   = 0,
    UNIT_OCCUPIED = 1,
} unit_occupancy_t;

/* Sensor model type — stored as uint8 in sensor_t for NVS round-trip */
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

/**
 * @brief Per-sensor persisted and runtime data.
 *
 * ZG-204ZV          : presence (IAS alarm1), temp, humidity, battery
 * ZG-205Z/A         : presence (IAS alarm1 + occupancy attribute)
 * ZG-102Z / ZG-102ZA: contact (IAS alarm1), tamper, battery_low, battery
 *
 * NOTE: illuminance_raw removed — not parsed (too many updates, not needed).
 *       sensor_role added — auto-assigned from model on identification.
 *       online added     — health status managed by watchdog.
 */
typedef struct {
    /* Identity */
    char     ieee_addr[IEEE_ADDR_STR_LEN];
    uint16_t short_addr;
    uint8_t  endpoint;
    char     sensor_name[SENSOR_NAME_LEN];
    uint8_t  sensor_type;       /* sensor_type_t stored as uint8 for NVS  */
    uint8_t  sensor_role;       /* sensor_role_t stored as uint8 for NVS  */

    /* Health */
    bool     online;            /* true = actively communicating           */
    uint8_t  ping_attempts;     /* watchdog retry counter                  */

    /* Presence / contact */
    bool     presence;          /* ZG-204ZV, ZG-205Z/A: true = occupied    */
    bool     contact_open;      /* ZG-102Z/A: true = door open             */
    bool     tamper;
    bool     battery_low;

    /* Environmental — ZG-204ZV only */
    int16_t  temperature_cdeg;  /* °C × 100  e.g. 2150 = 21.50 °C         */
    uint16_t humidity_cpct;     /* % × 100   e.g. 5000 = 50.00 %          */

    /* Battery — ZG-204ZV and ZG-102Z/A */
    uint8_t  battery_pct;       /* 0–100 % (clamped on write)              */

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
    /* Unit occupancy state machine */
    unit_occupancy_t unit_state;           /* VACANT or OCCUPIED              */
    time_t           unit_state_changed;   /* epoch when state last changed   */
    bool             door_closed_pending;  /* door closed, awaiting evaluation*/

    /* Hub aggregate presence (raw mmWave OR across all online sensors) */
    hub_status_t hub_status;

    /* Sensor registry */
    sensor_t sensors[MAX_SENSORS];
    uint8_t  sensor_count;

    hub_mode_t mode;
    bool       pairing_active;
    time_t     pairing_started;
} hub_config_t;

typedef struct {
    hub_config_t      data;
    SemaphoreHandle_t mutex;
} hub_config_safe_t;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

/* Thread-safe config access — always unlock after lock */
hub_config_t *lock_config(void);
void          unlock_config(void);

/* NVS persistence */
esp_err_t save_config(hub_config_t *config);
esp_err_t load_config(hub_config_t *config);

/* Mark config dirty so persist task saves on next tick.
 * Called from main.c and uart_hooks.c */
void mark_dirty(void);

/* Return human-readable model name string for a sensor type.
 * Used by uart_master.c when building JSON responses. */
const char *friendly_name_from_type(sensor_type_t t);

/* Unit occupancy state string — "OCCUPIED" or "VACANT" */
const char *unit_state_str(unit_occupancy_t s);

#endif /* MAIN_H */