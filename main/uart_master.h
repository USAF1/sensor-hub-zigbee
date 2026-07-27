/*
 * uart_master.h
 * Sensor Hub <-> Master UART Communication Layer
 * Innovatsii EMS — Pico 1
 *
 * Hardware connections:
 *   ESP32-C6 Sensor Hub:
 *     GPIO4  = TX → Master GPIO17 (RX)
 *     GPIO5  = RX ← Master GPIO16 (TX)
 *     GPIO16, GPIO17 = console UART (do not use for Master comms)
 *   Master ESP32-S3:
 *     GPIO16 = TX → Hub GPIO5 (RX)
 *     GPIO17 = RX ← Hub GPIO4 (TX)
 *
 * Baud rate: 9600 — reduced from 115200 for EMI noise immunity
 *   (802.15.4 Zigbee radio couples onto UART RX pin during TX burst)
 */

#ifndef UART_MASTER_H
#define UART_MASTER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// UART HARDWARE CONFIGURATION
// ============================================================================

#define UART_MASTER_PORT     UART_NUM_1
#define UART_MASTER_TX_PIN   4
#define UART_MASTER_RX_PIN   5
#define UART_MASTER_BAUD     9600

// ============================================================================
// BUFFER AND QUEUE SIZING
//
// TX_MSG_SIZE = 512:
//   Needed for config_response with MAX_SENSORS entries.
//   Each sensor entry is ~80 bytes. Header is ~150 bytes.
//   2 sensors: 150 + 80 + 80 + 4 (close) = 314 — fits in 512.
//   15 sensors worst case: 150 + (15 × 80) + 4 = 1354 — truncated
//   gracefully by the CR() macro's bounds check.
//
// TX_MSG_SIZE must match on both Hub and Master sides.
// Master (MicroPython) uses ujson.dumps() which produces spaced JSON.
// Hub (C) uses snprintf which produces compact JSON.
// Both are valid — json field extractors skip whitespace after ':'.
// ============================================================================

#define UART_MASTER_RX_BUF_SIZE    1024
#define UART_MASTER_TX_BUF_SIZE    0
#define UART_MASTER_LINE_BUF_SIZE  512
#define UART_MASTER_TX_MSG_SIZE    512
#define UART_MASTER_TX_QUEUE_DEPTH 16

// ============================================================================
// TASK STACK SIZES
//
// uart_tx_task:  2048 bytes is sufficient — tx_msg_t dequeue buffer is
//                now STATIC (s_tx_dequeue_buf), not on the stack.
//                Without this fix, the 514-byte tx_msg_t local variable
//                plus uart_write_bytes() internals overflows 2048 bytes,
//                causing a stack protection fault at ~29 minutes runtime
//                (when the first large heartbeat message is dequeued).
//
// uart_rx_task:  3584 bytes — needs 128-byte chunk buf + 80-byte hex_line
//                + FreeRTOS context + dispatch_command() call chain.
//
// hub_boot_retry: 4096 bytes — calls tx_send_fmt() which has a 512-byte
//                local buf[TX_MSG_SIZE] on stack + vsnprintf internals
//                + debug_print_tx() (hex_buf is static, not on stack)
//                + FreeRTOS context + safety margin.
//
// uart_tmr_task: 2560 bytes — calls tx_send_fmt() (512-byte local buf)
//                + door_alarm and heartbeat call chains.
// ============================================================================

#define UART_MASTER_RX_TASK_STACK   3584
#define UART_MASTER_TX_TASK_STACK   2048
#define UART_MASTER_RX_TASK_PRIO    4
#define UART_MASTER_TX_TASK_PRIO    4
#define UART_MASTER_TMR_TASK_STACK  2560
#define UART_MASTER_TMR_TASK_PRIO   3

// ============================================================================
// CONFIGURABLE PARAMETERS
// ============================================================================

typedef struct {
    uint16_t pairing_duration_sec;
    bool     watchdog_enable;
    uint16_t watchdog_interval_min;
    uint16_t watchdog_ping_timeout_sec;
    uint16_t door_alarm_threshold_min;
    uint16_t heartbeat_interval_min;
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
// ============================================================================

void uart_master_send_hub_boot(void);
void uart_master_send_hub_ready(void);
void uart_master_send_unit_occupancy(const char *state);
void uart_master_send_sensor_presence(const char *sensor_name,
                                      const char *model,
                                      bool        presence);
void uart_master_send_environment(const char *sensor_name,
                                  float        temp_c,
                                  float        humidity_pct);
void uart_master_send_door(const char *sensor_name, bool is_open);
void uart_master_notify_door_state(int         sensor_idx,
                                   const char *sensor_name,
                                   bool        is_open);
void uart_master_send_door_alarm(const char *sensor_name,
                                 const char *alarm_state,
                                 uint32_t    duration_sec);
void uart_master_send_sensor_health(const char *sensor_name,
                                    bool        is_online);
void uart_master_send_battery(const char *sensor_name,
                               uint8_t     battery_pct);
void uart_master_send_heartbeat(void);
void uart_master_send_config_response(void);
void uart_master_send_log_response(const char *log_line);
void uart_master_send_ack(const char *command, bool success);

#endif /* UART_MASTER_H */