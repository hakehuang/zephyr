/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_IMX_CCM_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_IMX_CCM_H_

#define IMX_CCM_CORESYS_CLK		0
#define IMX_CCM_PLATFORM_CLK		1
#define IMX_CCM_BUS_CLK			2
#define IMX_CCM_LPUART_CLK		3
#define IMX_CCM_LPI2C_CLK		4
#define IMX_CCM_LPSPI_CLK		5
#define IMX_CCM_USDHC1_CLK		6
#define IMX_CCM_USDHC2_CLK		7
#define IMX_CCM_EDMA_CLK		8
#define IMX_CCM_SAI1_CLK		9
#define IMX_CCM_SAI2_CLK		10
#define IMX_CCM_SAI3_CLK		11

#define IMX_CCM_CS1CDR_PREDIV   30
#define IMX_CCM_CS1CDR_PODF     31
#define IMX_CCM_CS2CDR_PREDIV   32
#define IMX_CCM_CS2CDR_PODF     33

#define IMX_CCM_AUDIO_BYPASS_SRC 40 
#define IMX_CCM_AUDIO_DIV_SEL    41
#define IMX_CCM_AUDIO_POST_DIV_SEL 42

#define IMX_CCM_ANALOG_PLL_AUDIO_NUM 50
#define IMX_CCM_ANALOG_PLL_AUDIO_DENOM 51


#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_IMX_CCM_H_ */
