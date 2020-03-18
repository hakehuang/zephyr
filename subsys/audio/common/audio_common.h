/*
 * Copyright (c) 2020 NXP Semiconductor INC.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AUDIO_COMMON_H_
#define _AUDIO_COMMON_H_

#include <audio/audio_core.h>

#if defined(__cplusplus)
extern "C" {
#endif

int audio_playback(struct audio_device_info *p_ad_info,
		   struct audio_device_par *param);

int audio_record(struct audio_device_info *p_ad_info,
		 struct audio_device_par *param);

int audio_loopback(struct audio_device_info *p_ad_info,
		   struct audio_device_par *param);

int audio_pause(struct audio_device_info *p_ad_info,
		struct audio_device_par *param);

int audio_stop(struct audio_device_info *p_ad_info,
	       struct audio_device_par *param);

int audio_vol(struct audio_device_info *p_ad_info,
	      struct audio_device_par *param);

int audio_mute(struct audio_device_info *p_ad_info,
	       struct audio_device_par *param);

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_CODEC_COMMON_H_ */
