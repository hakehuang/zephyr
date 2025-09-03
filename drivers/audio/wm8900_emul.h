/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Emulator API for Wolfson WM8900 Audio Codec
 */

#ifndef ZEPHYR_DRIVERS_AUDIO_WM8900_EMUL_H_
#define ZEPHYR_DRIVERS_AUDIO_WM8900_EMUL_H_

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reset the WM8900 emulator to default state
 * 
 * @param target The emulator target
 */
void wm8900_emul_reset(const struct emul *target);

/**
 * @brief Set a register value in the WM8900 emulator
 * 
 * @param target The emulator target
 * @param reg_addr Register address
 * @param value 16-bit register value
 */
void wm8900_emul_set_reg(const struct emul *target, uint8_t reg_addr, uint16_t value);

/**
 * @brief Get a register value from the WM8900 emulator
 * 
 * @param target The emulator target
 * @param reg_addr Register address
 * @return uint16_t Register value
 */
uint16_t wm8900_emul_get_reg(const struct emul *target, uint8_t reg_addr);

/**
 * @brief Set volume level for testing
 * 
 * @param target The emulator target
 * @param channel Audio channel (left/right/all)
 * @param volume Volume level (0-100)
 * @return int 0 on success, negative error code on failure
 */
int wm8900_emul_set_volume(const struct emul *target, uint8_t channel, uint8_t volume);

/**
 * @brief Set mute state for testing
 * 
 * @param target The emulator target
 * @param channel Audio channel (left/right/all)
 * @param muted True to mute, false to unmute
 * @return int 0 on success, negative error code on failure
 */
int wm8900_emul_set_mute(const struct emul *target, uint8_t channel, bool muted);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_AUDIO_WM8900_EMUL_H_ */