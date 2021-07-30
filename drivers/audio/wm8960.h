/*
 * Copyright (c) 2021 NXP Semiconductor INC.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _CODEC_WM8960_H_
#define _CODEC_WM8960_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Define the register address of WM8960. */
#define WM8960_LINVOL 0x0
#define WM8960_RINVOL 0x1
#define WM8960_LOUT1 0x2
#define WM8960_ROUT1 0x3
#define WM8960_CLOCK1 0x4
#define WM8960_DACCTL1 0x5
#define WM8960_DACCTL2 0x6
#define WM8960_IFACE1 0x7
#define WM8960_CLOCK2 0x8
#define WM8960_IFACE2 0x9
#define WM8960_LDAC 0xa
#define WM8960_RDAC 0xb

#define WM8960_RESET 0xf
#define WM8960_3D 0x10
#define WM8960_ALC1 0x11
#define WM8960_ALC2 0x12
#define WM8960_ALC3 0x13
#define WM8960_NOISEG 0x14
#define WM8960_LADC 0x15
#define WM8960_RADC 0x16
#define WM8960_ADDCTL1 0x17
#define WM8960_ADDCTL2 0x18
#define WM8960_POWER1 0x19
#define WM8960_POWER2 0x1a
#define WM8960_ADDCTL3 0x1b
#define WM8960_APOP1 0x1c
#define WM8960_APOP2 0x1d

#define WM8960_LINPATH 0x20
#define WM8960_RINPATH 0x21
#define WM8960_LOUTMIX 0x22

#define WM8960_ROUTMIX 0x25
#define WM8960_MONOMIX1 0x26
#define WM8960_MONOMIX2 0x27
#define WM8960_LOUT2 0x28
#define WM8960_ROUT2 0x29
#define WM8960_MONO 0x2a
#define WM8960_INBMIX1 0x2b
#define WM8960_INBMIX2 0x2c
#define WM8960_BYPASS1 0x2d
#define WM8960_BYPASS2 0x2e
#define WM8960_POWER3 0x2f
#define WM8960_ADDCTL4 0x30
#define WM8960_CLASSD1 0x31

#define WM8960_CLASSD3 0x33
#define WM8960_PLL1 0x34
#define WM8960_PLL2 0x35
#define WM8960_PLL3 0x36
#define WM8960_PLL4 0x37

/*! @brief Cache register number */
#define WM8960_CACHEREGNUM 56

/*! @brief WM8960_IFACE1 FORMAT bits */
#define WM8960_IFACE1_FORMAT_MASK 0x03
#define WM8960_IFACE1_FORMAT_SHIFT 0x00
#define WM8960_IFACE1_FORMAT_RJ 0x00
#define WM8960_IFACE1_FORMAT_LJ 0x01
#define WM8960_IFACE1_FORMAT_I2S 0x02
#define WM8960_IFACE1_FORMAT_DSP 0x03
#define WM8960_IFACE1_FORMAT(x) ((x << WM8960_IFACE1_FORMAT_SHIFT) & WM8960_IFACE1_FORMAT_MASK)

/*! @brief WM8960_IFACE1 WL bits */
#define WM8960_IFACE1_WL_MASK 0x0C
#define WM8960_IFACE1_WL_SHIFT 0x02
#define WM8960_IFACE1_WL_16BITS 0x00
#define WM8960_IFACE1_WL_20BITS 0x01
#define WM8960_IFACE1_WL_24BITS 0x02
#define WM8960_IFACE1_WL_32BITS 0x03
#define WM8960_IFACE1_WL(x) ((x << WM8960_IFACE1_WL_SHIFT) & WM8960_IFACE1_WL_MASK)

/*! @brief WM8960_IFACE1 LRP bit */
#define WM8960_IFACE1_LRP_MASK 0x10
#define WM8960_IFACE1_LRP_SHIFT 0x04
#define WM8960_IFACE1_LRCLK_NORMAL_POL 0x00
#define WM8960_IFACE1_LRCLK_INVERT_POL 0x01
#define WM8960_IFACE1_DSP_MODEA 0x00
#define WM8960_IFACE1_DSP_MODEB 0x01
#define WM8960_IFACE1_LRP(x) ((x << WM8960_IFACE1_LRP_SHIFT) & WM8960_IFACE1_LRP_MASK)

/*! @brief WM8960_IFACE1 DLRSWAP bit */
#define WM8960_IFACE1_DLRSWAP_MASK 0x20
#define WM8960_IFACE1_DLRSWAP_SHIFT 0x05
#define WM8960_IFACE1_DACCH_NORMAL 0x00
#define WM8960_IFACE1_DACCH_SWAP 0x01
#define WM8960_IFACE1_DLRSWAP(x) ((x << WM8960_IFACE1_DLRSWAP_SHIFT) & WM8960_IFACE1_DLRSWAP_MASK)

/*! @brief WM8960_IFACE1 MS bit */
#define WM8960_IFACE1_MS_MASK 0x40
#define WM8960_IFACE1_MS_SHIFT 0x06
#define WM8960_IFACE1_SLAVE 0x00
#define WM8960_IFACE1_MASTER 0x01
#define WM8960_IFACE1_MS(x) ((x << WM8960_IFACE1_MS_SHIFT) & WM8960_IFACE1_MS_MASK)

/*! @brief WM8960_IFACE1 BCLKINV bit */
#define WM8960_IFACE1_BCLKINV_MASK 0x80
#define WM8960_IFACE1_BCLKINV_SHIFT 0x07
#define WM8960_IFACE1_BCLK_NONINVERT 0x00
#define WM8960_IFACE1_BCLK_INVERT 0x01
#define WM8960_IFACE1_BCLKINV(x) ((x << WM8960_IFACE1_BCLKINV_SHIFT) & WM8960_IFACE1_BCLKINV_MASK)

/*! @brief WM8960_IFACE1 ALRSWAP bit */
#define WM8960_IFACE1_ALRSWAP_MASK 0x100
#define WM8960_IFACE1_ALRSWAP_SHIFT 0x08
#define WM8960_IFACE1_ADCCH_NORMAL 0x00
#define WM8960_IFACE1_ADCCH_SWAP 0x01
#define WM8960_IFACE1_ALRSWAP(x) ((x << WM8960_IFACE1_ALRSWAP_SHIFT) & WM8960_IFACE1_ALRSWAP_MASK)

/*! @brief WM8960_POWER1 */
#define WM8960_POWER1_VREF_MASK 0x40
#define WM8960_POWER1_VREF_SHIFT 0x06

#define WM8960_POWER1_AINL_MASK 0x20
#define WM8960_POWER1_AINL_SHIFT 0x05

#define WM8960_POWER1_AINR_MASK 0x10
#define WM8960_POWER1_AINR_SHIFT 0x04

#define WM8960_POWER1_ADCL_MASK 0x08
#define WM8960_POWER1_ADCL_SHIFT 0x03

#define WM8960_POWER1_ADCR_MASK 0x0
#define WM8960_POWER1_ADCR_SHIFT 0x02

#define WM8960_POWER1_MICB_MASK 0x02
#define WM8960_POWER1_MICB_SHIFT 0x01

#define WM8960_POWER1_DIGENB_MASK 0x01
#define WM8960_POWER1_DIGENB_SHIFT 0x00

/*! @brief WM8960_POWER2 */
#define WM8960_POWER2_DACL_MASK 0x100
#define WM8960_POWER2_DACL_SHIFT 0x08

#define WM8960_POWER2_DACR_MASK 0x80
#define WM8960_POWER2_DACR_SHIFT 0x07

#define WM8960_POWER2_LOUT1_MASK 0x40
#define WM8960_POWER2_LOUT1_SHIFT 0x06

#define WM8960_POWER2_ROUT1_MASK 0x20
#define WM8960_POWER2_ROUT1_SHIFT 0x05

#define WM8960_POWER2_SPKL_MASK 0x10
#define WM8960_POWER2_SPKL_SHIFT 0x04

#define WM8960_POWER2_SPKR_MASK 0x08
#define WM8960_POWER2_SPKR_SHIFT 0x03

#define WM8960_POWER3_LMIC_MASK 0x20
#define WM8960_POWER3_LMIC_SHIFT 0x05
#define WM8960_POWER3_RMIC_MASK 0x10
#define WM8960_POWER3_RMIC_SHIFT 0x04
#define WM8960_POWER3_LOMIX_MASK 0x08
#define WM8960_POWER3_LOMIX_SHIFT 0x03
#define WM8960_POWER3_ROMIX_MASK 0x04
#define WM8960_POWER3_ROMIX_SHIFT 0x02
/*! @brief WM8960 I2C address. */
#define WM8960_I2C_ADDR 0x1A
/*! @brief WM8960 I2C baudrate */
#define WM8960_I2C_BAUDRATE (100000U)

/*! @brief Modules in WM8960 board. */
enum _wm8960_module {
	kWM8960_ModuleADC  = 0, /*!< ADC module in WM8960 */
	kWM8960_ModuleDAC  = 1, /*!< DAC module in WM8960 */
	kWM8960_ModuleVREF = 2, /*!< VREF module */
	kWM8960_ModuleHP   = 3, /*!< Headphone */
	kWM8960_ModuleMICB = 4, /*!< Mic bias */
	kWM8960_ModuleMIC  = 5, /*!< Input Mic */
	kWM8960_ModuleLineIn  = 6, /*!< Analog in PGA  */
	kWM8960_ModuleLineOut = 7, /*!< Line out module */
	kWM8960_ModuleSpeaker = 8, /*!< Speaker module */
	kWM8960_ModuleOMIX    = 9, /*!< Output mixer */
};

/*! @brief wm8960 play channel */
enum _wm8960_play_channel {
	kWM8960_HeadphoneLeft  = 1, /*!< wm8960 headphone left channel */
	kWM8960_HeadphoneRight = 2, /*!< wm8960 headphone right channel */
	kWM8960_SpeakerLeft    = 4, /*!< wm8960 speaker left channel */
	kWM8960_SpeakerRight   = 8, /*!< wm8960 speaker right channel */
};

/*! @brief wm8960 play source */
enum _wm8960_play_source {
	kWM8960_PlaySourcePGA   = 1, /*!< wm8960 play source PGA */
	kWM8960_PlaySourceInput = 2, /*!< wm8960 play source Input */
	kWM8960_PlaySourceDAC   = 4, /*!< wm8960 play source DAC */
};

/*!
 * @brief WM8960 data route.
 * Only provide some typical data route, not all route listed.
 * Note: Users cannot combine any routes, once a new route is set,
 * the previous one would be replaced.
 */
enum _wm8960_route {
	kWM8960_RouteBypass            = 0, /*!< LINEIN->Headphone. */
	kWM8960_RoutePlayback          = 1, /*!<  I2SIN->DAC->Headphone. */
	kWM8960_RoutePlaybackandRecord = 2, /*!< I2SIN->DAC->Headphone, LINEIN->ADC->I2SOUT. */
	kWM8960_RouteRecord            = 5  /*!< LINEIN->ADC->I2SOUT. */
};

/*!
 * @brief The audio data transfer protocol choice.
 * WM8960 only supports I2S format and PCM format.
 */
enum _wm8960_protocol {
	kWM8960_BusI2S            = 2,           /*!< I2S type */
	kWM8960_BusLeftJustified  = 1,           /*!< Left justified mode */
	kWM8960_BusRightJustified = 0,           /*!< Right justified mode */
	kWM8960_BusPCMA           = 3,           /*!< PCM A mode */
	kWM8960_BusPCMB           = 3 | (1 << 4) /*!< PCM B mode */
};

/*! @brief wm8960 input source */
enum _wm8960_input {
	kWM8960_InputClosed = 0,
	kWM8960_InputSingleEndedMic        = 1,
	kWM8960_InputDifferentialMicInput2 = 2,
	kWM8960_InputDifferentialMicInput3 = 3,
	kWM8960_InputLineINPUT2            = 4,
	kWM8960_InputLineINPUT3            = 5
};

/*! @brief audio sample rate definition */
enum _wm8960_sample_rate {
	kWM8960_AudioSampleRate8KHz    = 8000U,   /*!< Sample rate 8000 Hz */
	kWM8960_AudioSampleRate11025Hz = 11025U,  /*!< Sample rate 11025 Hz */
	kWM8960_AudioSampleRate12KHz   = 12000U,  /*!< Sample rate 12000 Hz */
	kWM8960_AudioSampleRate16KHz   = 16000U,  /*!< Sample rate 16000 Hz */
	kWM8960_AudioSampleRate22050Hz = 22050U,  /*!< Sample rate 22050 Hz */
	kWM8960_AudioSampleRate24KHz   = 24000U,  /*!< Sample rate 24000 Hz */
	kWM8960_AudioSampleRate32KHz   = 32000U,  /*!< Sample rate 32000 Hz */
	kWM8960_AudioSampleRate44100Hz = 44100U,  /*!< Sample rate 44100 Hz */
	kWM8960_AudioSampleRate48KHz   = 48000U,  /*!< Sample rate 48000 Hz */
	kWM8960_AudioSampleRate96KHz   = 96000U,  /*!< Sample rate 96000 Hz */
	kWM8960_AudioSampleRate192KHz  = 192000U, /*!< Sample rate 192000 Hz */
	kWM8960_AudioSampleRate384KHz  = 384000U, /*!< Sample rate 384000 Hz */
};

/*! @brief audio bit width */
enum _wm8960_audio_bit_width {
	kWM8960_AudioBitWidth16bit = 16U, /*!< audio bit width 16 */
	kWM8960_AudioBitWidth20bit = 20U, /*!< audio bit width 20 */
	kWM8960_AudioBitWidth24bit = 24U, /*!< audio bit width 24 */
	kWM8960_AudioBitWidth32bit = 32U, /*!< audio bit width 32 */
};

/*! @brief wm8960 audio format */
struct _wm8960_audio_format {
	uint32_t mclk_HZ;    /*!< master clock frequency */
	uint32_t sampleRate; /*!< sample rate */
	uint32_t bitWidth;   /*!< bit width */
};


/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif
