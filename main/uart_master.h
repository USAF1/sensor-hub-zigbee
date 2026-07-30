/*
 * uart_master.h
 * Sensor Hub <-> Master UART Communication Layer
 * Innovatsii EMS — Pico 1
 * Firmware Version: 0.2.5
 *
 * Hardware connections:
 *   ESP32-C6 Sensor Hub:
 *     GPIO4  = TX → Master GPIO17 (RX)
 *     GPIO5  = RX ← Master GPIO16 (TX)
 *     GPIO16, GPIO17 = console UART (do not use for Master comms)
 *
 * Baud rate: 9600 — reduced from 115200 for EMI noise immunity
 *   (802.15.4 Zigbee radio couples onto UART RX pin during TX burst)
 *
 * Changes in v0.2.5:
 *   - uart_hub_config_t: added presence_fading_time_sec,
 *     door_sensor_max_silence_hours
 *   - New boot protocol: ping/pong, hub_init, start_watchdog
 *   - New outbound APIs: pong, sensor_joined, sensor_status,
 *     sensor_list_complete, new_sensor_joined, pairing_complete
 *   - hub_boot_retry task replaced by passive ping responder
 *   - hub_boot_retry stack removed (no longer needed)
 *   - TMR task stack increased to 3072 (calls fading time sender)
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
// ============================================================================

#define UART_MASTER_RX_BUF_SIZE    1024
#define UART_MASTER_TX_BUF_SIZE    0
#define UART_MASTER_LINE_BUF_SIZE  512
#define UART_MASTER_TX_MSG_SIZE    512
#define UART_MASTER_TX_QUEUE_DEPTH 16

// ============================================================================
// TASK STACK SIZES
//
// uart_tx_task: 2048 — s_tx_dequeue_buf is STATIC global (not on stack).
//   CRITICAL: never change s_tx_dequeue_buf back to a local variable.
//   A 514-byte local tx_msg_t overflows 2048-byte stack at ~29 min runtime.
//
// uart_rx_task: 3584 — 128-byte chunk + dispatch_command() call chain.
//
// uart_tmr_task: 3072 — calls tx_send_fmt (512-byte buf on stack) +
//   door_alarm + heartbeat + 24h door silence check. Increased from 2560.
// ============================================================================

#define UART_MASTER_RX_TASK_STACK   3584
#define UART_MASTER_TX_TASK_STACK   2048
#define UART_MASTER_RX_TASK_PRIO    4
#define UART_MASTER_TX_TASK_PRIO    4
#define UART_MASTER_TMR_TASK_STACK  3072
#define UART_MASTER_TMR_TASK_PRIO   3

// ============================================================================
// CONFIGURABLE PARAMETERS
//
// presence_fading_time_sec:
//   How long ZG-204ZV holds presence=YES after last detection.
//   Default 30s. Prevents YES/NO oscillation when person is near
//   the detection boundary. Set via Tuya EF00 after every join.
//
// door_sensor_max_silence_hours:
//   Hours without any report from a door sensor before a
//   HEALTH OFFLINE alert is sent. Default 24h.
//   Alert is notification only — no pairing window opens.
// ============================================================================

typedef struct {
    uint16_t pairing_duration_sec;
    bool     watchdog_enable;
    uint16_t watchdog_interval_min;
    uint16_t watchdog_ping_timeout_sec;
    uint16_t door_alarm_threshold_min;
    uint16_t heartbeat_interval_min;
    uint16_t presence_fading_time_sec;       /* v0.2.5: ZG-204ZV fading hold */
    uint16_t door_sensor_max_silence_hours;  /* v0.2.5: door silence alert   */
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
// OUTBOUND MESSAGE API — Boot Protocol (v0.2.5)
// ============================================================================

/* Respond to Master ping */
void uart_master_send_pong(void);

/* Sent after Zigbee network formed and NVS loaded */
void uart_master_send_hub_ready(void);

/* Sent for each sensor that successfully rejoins at boot */
void uart_master_send_sensor_joined(int         sensor_idx,
                                    const char *name,
                                    const char *model,
                                    const char *role,
                                    bool        online,
                                    uint8_t     battery_pct);

/* Sent for each sensor that fails to rejoin after all retries */
void uart_master_send_sensor_status(int         sensor_idx,
                                    const char *name,
                                    const char *model,
                                    const char *role,
                                    bool        online);

/* Sent after all rejoin attempts complete */
void uart_master_send_sensor_list_complete(int total, int online, int offline);

/* Sent when a new sensor joins during pairing window */
void uart_master_send_new_sensor_joined(int         sensor_idx,
                                        const char *name,
                                        const char *model,
                                        const char *role);

/* Sent when pairing window closes */
void uart_master_send_pairing_complete(int new_sensors, int total_sensors);

// ============================================================================
// OUTBOUND MESSAGE API — Runtime
// ============================================================================

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