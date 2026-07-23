/*
 * uart_master.h
 * Sensor Hub <-> Master UART Communication Layer
 * Innovatsii EMS — Pico 1
 *
 * Production rules:
 *   - All buffers statically allocated — zero heap in steady state
 *   - Transmit via FreeRTOS queue (non-blocking for all callers)
 *   - Receive via dedicated task with fixed line buffer
 *   - JSON parser handles both compact and spaced key:value forms
 *   - Command dispatch via table — adding a command = one table entry
 */

#ifndef UART_MASTER_H
#define UART_MASTER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// UART HARDWARE CONFIGURATION
// ESP32-C6 Sensor Hub:
//   GPIO16, GPIO17 = console UART (do not use)
//   GPIO4  = TX to Master RX (GPIO17 on ESP32-S3)
//   GPIO5  = RX from Master TX (GPIO16 on ESP32-S3)
// ============================================================================

#define UART_MASTER_PORT     UART_NUM_1
#define UART_MASTER_TX_PIN   4
#define UART_MASTER_RX_PIN   5
#define UART_MASTER_BAUD     115200

// ============================================================================
// BUFFER AND QUEUE SIZING
// ============================================================================

#define UART_MASTER_RX_BUF_SIZE    1024   /* UART driver RX ring buffer      */
#define UART_MASTER_TX_BUF_SIZE    0      /* TX handled synchronously        */
#define UART_MASTER_LINE_BUF_SIZE  512    /* max bytes in one inbound line   */
#define UART_MASTER_TX_MSG_SIZE    256    /* max bytes in one outbound msg   */
#define UART_MASTER_TX_QUEUE_DEPTH 16     /* outbound message slots          */

// ============================================================================
// TASK CONFIGURATION
// ============================================================================

#define UART_MASTER_RX_TASK_STACK  3072
#define UART_MASTER_TX_TASK_STACK  2048
#define UART_MASTER_RX_TASK_PRIO   4
#define UART_MASTER_TX_TASK_PRIO   4
#define UART_MASTER_TMR_TASK_STACK 2048
#define UART_MASTER_TMR_TASK_PRIO  3

// ============================================================================
// CONFIGURABLE PARAMETERS
// Stored in NVS. Pushed from Master via set_config command.
// All values validated and range-clamped on write.
// ============================================================================

typedef struct {
    uint16_t pairing_duration_sec;       /* 30  – 600   default: 120 */
    bool     watchdog_enable;            /* default: true             */
    uint16_t watchdog_interval_min;      /* 1   – 1440  default: 60  */
    uint16_t watchdog_ping_timeout_sec;  /* 10  – 120   default: 30  */
    uint16_t door_alarm_threshold_min;   /* 1   – 60    default: 10  */
    uint16_t heartbeat_interval_min;     /* 1   – 1440  default: 30  */
} uart_hub_config_t;

// ============================================================================
// INIT
// ============================================================================

esp_err_t uart_master_init(void);

// ============================================================================
// CONFIG API
// ============================================================================

void      uart_master_get_config(uart_hub_config_t *out);
esp_err_t uart_master_set_config(const uart_hub_config_t *in);
esp_err_t uart_master_load_config(void);
esp_err_t uart_master_save_config(void);

// ============================================================================
// OUTBOUND MESSAGE API
// All functions non-blocking — messages queued to TX task.
// Queue full = message dropped with warning log.
// ============================================================================

/* Boot notification — sent immediately on app_main start */
void uart_master_send_hub_boot(void);

/* Sent when pairing window closes — summary of who is online */
void uart_master_send_hub_ready(void);

/* Unit occupancy changed */
void uart_master_send_unit_occupancy(const char *state);

/* Individual sensor presence changed */
void uart_master_send_sensor_presence(const char *sensor_name,
                                      const char *model,
                                      bool        presence);

/* Temperature or humidity changed */
void uart_master_send_environment(const char *sensor_name,
                                  float        temp_c,
                                  float        humidity_pct);

/* Door open or close event */
void uart_master_send_door(const char *sensor_name, bool is_open);

/* Door alarm tracker — call on every door state change */
void uart_master_notify_door_state(int         sensor_idx,
                                   const char *sensor_name,
                                   bool        is_open);

/* Door alarm exceeded threshold / cleared */
void uart_master_send_door_alarm(const char *sensor_name,
                                 const char *alarm_state,
                                 uint32_t    duration_sec);

/* Sensor health — online or offline */
void uart_master_send_sensor_health(const char *sensor_name,
                                    bool        is_online);

/* Battery level update */
void uart_master_send_battery(const char *sensor_name,
                               uint8_t     battery_pct);

/* Periodic heartbeat */
void uart_master_send_heartbeat(void);

/* Config response — reply to get_config */
void uart_master_send_config_response(void);

/* Log dump response — reply to get_logs */
void uart_master_send_log_response(const char *log_line);

/* ACK — reply to any command */
void uart_master_send_ack(const char *command, bool success);

#endif /* UART_MASTER_H */