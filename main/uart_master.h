/*
 * uart_master.h
 * Sensor Hub <-> Master UART Communication Layer
 * Innovatsii EMS — Pico 1
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
#define UART_MASTER_BAUD     9600

// ============================================================================
// BUFFER AND QUEUE SIZING
// ============================================================================

#define UART_MASTER_RX_BUF_SIZE    1024
#define UART_MASTER_TX_BUF_SIZE    0
#define UART_MASTER_LINE_BUF_SIZE  512
#define UART_MASTER_TX_MSG_SIZE    512
#define UART_MASTER_TX_QUEUE_DEPTH 16

// ============================================================================
// TASK CONFIGURATION
// ============================================================================

#define UART_MASTER_RX_TASK_STACK  3584
#define UART_MASTER_TX_TASK_STACK  2048
#define UART_MASTER_RX_TASK_PRIO   4
#define UART_MASTER_TX_TASK_PRIO   4
#define UART_MASTER_TMR_TASK_STACK 2048
#define UART_MASTER_TMR_TASK_PRIO  3

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