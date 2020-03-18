/*
 * Copyright (c) 2020 NXP Semiconductor INC.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <string.h>

#define LOG_LEVEL CONFIG_SOC_AUDIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(soc_audio);

#include "audio_common.h"

int audio_playback(struct audio_device_info *p_ad_info,
		   struct audio_device_par *param) {
	return 0;
}

int audio_record(struct audio_device_info *p_ad_info,
		 struct audio_device_par *param) {
	return 0;
}

int audio_loopback(struct audio_device_info *p_ad_info,
		   struct audio_device_par *param) {
	return 0;
}

int audio_pause(struct audio_device_info *p_ad_info,
		struct audio_device_par *param) {
	return 0;
}

int audio_stop(struct audio_device_info *p_ad_info,
	       struct audio_device_par *param) {
	return 0;
}

int audio_vol(struct audio_device_info *p_ad_info,
	      struct audio_device_par *param) {
	return 0;
}

int audio_mute(struct audio_device_info *p_ad_info,
	       struct audio_device_par *param) {
	return 0;
}

