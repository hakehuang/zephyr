/*
 * Copyright (c) 2025 Hake Huang <hake.huang@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DT_BINDINGS_ADC_ADS1256_H_
#define ZEPHYR_DT_BINDINGS_ADC_ADS1256_H_

/* Data rate values (DRATE register settings) */

#define ADS1256_DRATE_30000_VAL     0xF0
#define ADS1256_DRATE_15000_VAL     0xE0
#define ADS1256_DRATE_7500_VAL      0xD0
#define ADS1256_DRATE_3750_VAL      0xC0
#define ADS1256_DRATE_2000_VAL      0xB0
#define ADS1256_DRATE_1000_VAL      0xA1
#define ADS1256_DRATE_500_VAL       0x92
#define ADS1256_DRATE_100_VAL       0x82
#define ADS1256_DRATE_60_VAL        0x72
#define ADS1256_DRATE_50_VAL        0x63
#define ADS1256_DRATE_30_VAL        0x53
#define ADS1256_DRATE_25_VAL        0x43
#define ADS1256_DRATE_15_VAL        0x33
#define ADS1256_DRATE_10_VAL        0x23
#define ADS1256_DRATE_5_VAL         0x13
#define ADS1256_DRATE_2P5_VAL       0x03

#define ADC_CALIBRATE_OFFSET0   0
#define ADC_CALIBRATE_GAIN1     1

#endif /* ZEPHYR_DT_BINDINGS_ADC_ADS1256_H_ */