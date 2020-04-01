/*
 * Copyright (c) 2020 NXP Semiconductor INC.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief I2S bus (SAI) driver for NXP i.MX RT series.
 *
 */

#ifndef ZEPHYR_DRIVERS_I2S_I2S_IMX_RT_H_
#define ZEPHYR_DRIVERS_I2S_I2S_IMX_RT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_sai_edma.h"
#include "fsl_sai.h"


#define SAI_WORD_SIZE_BITS_MIN 8
#define SAI_WORD_SIZE_BITS_MAX 32

#define SAI_WORD_PER_FRAME_MIN 0
#define SAI_WORD_PER_FRAME_MAX 32

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_I2S_I2S_IMX_RT_H_ */
