/*
 * uart_master.h
 * Sensor Hub <-> Master UART Communication Layer
 * Innovatsii EMS — Pico 1  |  Firmware 0.2.5
 *
 * V4.2+:
 *   - hub_aggregate replaces unit_occupancy
 *   - UTC timestamps on all messages (g_utc_boot_epoch + uptime)
 *   - presence_fading_time_sec: 0 is valid (no hold)
 *   - motion_sensitivity global default added
 *   - set_sensor_config command support (per-sensor fading/sensitivity)
 */

#ifndef UART_MASTER_H
#define UART_MASTER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// UART HARDWARE
// ============================================================================

#define UART_MASTER_PORT     UART_NUM_1
#define UART_MASTER_TX_PIN   4
#define UART_MASTER_RX_PIN   5
#define UART_MASTER_BAUD     9600

// ============================================================================
// BUFFER SIZING
// ============================================================================

#define UART_MASTER_RX_BUF_SIZE    1024
#define UART_MASTER_TX_BUF_SIZE    0
#define UART_MASTER_LINE_BUF_SIZE  512
#define UART_MASTER_TX_MSG_SIZE    1024
#define UART_MASTER_TX_QUEUE_DEPTH 16

// ============================================================================
// TASK STACK SIZES
// ============================================================================

#define UART_MASTER_RX_TASK_STACK   3584
#define UART_MASTER_TX_TASK_STACK   2048
#define UART_MASTER_RX_TASK_PRIO    4
#define UART_MASTER_TX_TASK_PRIO    4
#define UART_MASTER_TMR_TASK_STACK  3072
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
    uint16_t presence_fading_time_sec;       /* 0..28800; global default */
    uint16_t door_sensor_max_silence_hours;
    uint16_t motion_sensitivity;             /* 0..19; global default    */
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
// OUTBOUND — Boot Protocol
// ============================================================================

void uart_master_send_pong(void);
void uart_master_send_hub_ready(void);
void uart_master_send_sensor_joined(int sensor_idx, const char *name,
                                    const char *model, const char *role,
                                    bool online, uint8_t battery_pct);
void uart_master_send_sensor_status(int sensor_idx, const char *name,
                                    const char *model, const char *role,
                                    bool online);
void uart_master_send_sensor_list_complete(int total, int online, int offline);
void uart_master_send_new_sensor_joined(int sensor_idx, const char *name,
                                        const char *model, const char *role);
void uart_master_send_pairing_complete(int new_sensors, int total_sensors);

// ============================================================================
// OUTBOUND — Runtime
// ============================================================================

void uart_master_send_hub_aggregate(const char *state);
void uart_master_send_sensor_presence(const char *sensor_name,
                                      const char *model, bool presence);
void uart_master_send_door(const char *sensor_name, bool is_open);
void uart_master_notify_door_state(int sensor_idx, const char *sensor_name,
                                   bool is_open);
void uart_master_send_door_alarm(const char *sensor_name,
                                 const char *alarm_state,
                                 uint32_t duration_sec);
void uart_master_send_sensor_health(const char *sensor_name, bool is_online);
void uart_master_send_battery(const char *sensor_name, uint8_t battery_pct);
void uart_master_send_heartbeat(void);
void uart_master_send_config_response(void);
void uart_master_send_log_response(const char *log_line);
void uart_master_send_ack(const char *command, bool success);

#endif /* UART_MASTER_H */