/*
 * main.h — Sensor Hub Zigbee Coordinator
 * Innovatsii EMS — Pico 1
 * Firmware Version: 0.2.5
 *
 * Architecture per V4.2 spec:
 *   - Passive boot: waits for ping from Master, no autonomous Zigbee start
 *   - Sensor identification from bind confirmation + model ID read sequence
 *   - ZG-102Z classified from first IAS zone status if model ID times out
 *   - Door sensor NEVER marked offline by watchdog (sleepy device)
 *   - Presence sensor immediate offline alert
 *   - FACTORY_RESET_MODE = 0 always for production
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

/* Occupancy */
#define DOOR_PENDING_WINDOW_SEC  30

/* ZG-204ZV oscillation fix */
#define PRESENCE_FADING_TIME_DEFAULT_SEC 30

/*
 * MODEL_ID_TIMEOUT_MS:
 *   After bind confirmations arrive, we send the model ID read.
 *   If no response arrives within this window, the sensor is sleepy
 *   (ZG-102Z) and we classify from the first inbound data packet.
 */
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
    UNIT_VACANT   = 0,
    UNIT_OCCUPIED = 1,
} unit_occupancy_t;

typedef enum {
    SENSOR_UNKNOWN   = 0,
    SENSOR_ZG_204ZV  = 1,   /* mmWave: IAS presence + temp + hum + battery */
    SENSOR_ZG_205Z_A = 2,   /* mmWave: occupancy sensing + IAS */
    SENSOR_ZG_102Z   = 3,   /* Door: IAS contact + battery — sleepy */
    SENSOR_ZG_102ZA  = 4,   /* Door: IAS contact + battery — sleepy */
} sensor_type_t;

/*
 * Sensor identification state machine.
 * Each sensor progresses through these states after DEVICE_ANNCE.
 */
typedef enum {
    ID_STATE_UNIDENTIFIED  = 0,  /* Just joined, EP request not yet sent */
    ID_STATE_EP_REQUESTED  = 1,  /* Active EP request sent */
    ID_STATE_BINDING       = 2,  /* Bind requests sent, waiting for confirms */
    ID_STATE_MODEL_READ    = 3,  /* Bind confirmed, model ID read sent */
    ID_STATE_IDENTIFIED    = 4,  /* Model known, configure reporting sent */
    ID_STATE_OPERATIONAL   = 5,  /* All setup complete, data flowing */
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

/*
 * sensor_runtime_meta_t — volatile per-sensor state.
 * NOT persisted in NVS (reset on boot).
 * All state needed to track identification, binding, and watchdog.
 */
typedef struct {
    /* Identification state machine */
    sensor_id_state_t id_state;
    char              model_id[32];        /* model string from Basic cluster */
    bool              model_known;         /* true after model identified */

    /* Bind tracking */
    uint8_t           bind_pending;        /* bind requests sent not yet answered */
    uint8_t           bind_confirmed;      /* bind responses with status=OK */
    uint8_t           bind_failed;         /* bind responses with timeout/error */
    bool              bound_once;          /* IAS+OCC binds completed at least once */
    bool              power_config_bound;  /* POWER_CONFIG bound after identification */
    uint8_t           ep_active;           /* endpoint number from active EP response */

    /* Post-identification setup */
    bool              reporting_configured;
    bool              fade_sent;

    /* Model ID read timeout tracking */
    uint32_t          model_id_req_ms;     /* esp_timer ms when model ID was requested */
    bool              model_id_pending;    /* model ID read sent, waiting for response */

    /* Watchdog */
    bool              ping_pending;
    uint8_t           miss_count;
    uint8_t           rejoin_count;

    /* IAS zone enroll */
    bool              enroll_sent;
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