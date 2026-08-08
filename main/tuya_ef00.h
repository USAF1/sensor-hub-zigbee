/*
 * tuya_ef00.h — Tuya EF00 cluster write helpers
 * Innovatsii EMS — Pico 1  |  Firmware 0.3.0
 */

#ifndef TUYA_EF00_H
#define TUYA_EF00_H

#include <stdint.h>

/* Send a Tuya VALUE (4-byte big-endian uint32) datapoint write.
 * Spawns a tiny one-shot task that is self-deleting; the arg is freed on
 * completion or dropped (memory freed) if the task-create fails. */
void tuya_write_dp(uint16_t sa, uint8_t ep, uint8_t dp, uint32_t val);

/* Send a Tuya ENUM (1-byte) datapoint write.
 * Same one-shot task pattern as tuya_write_dp. */
void tuya_write_dp_enum(uint16_t sa, uint8_t ep, uint8_t dp, uint8_t val);

#endif /* TUYA_EF00_H */
