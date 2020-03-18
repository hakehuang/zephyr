/*
 * Copyright (c) 2020 NXP Semiconductor INC.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_AUDIO_CORE_H_
#define ZEPHYR_INCLUDE_AUDIO_CORE_H_

#include <kernel.h>
#include <zephyr/types.h>
#include <sys/dlist.h>

#ifdef __cplusplus
extern "C" {
#endif

/*defined in audio driver*/
struct audio_codec_api;
struct audio_codec_cfg;

/*defined in the dai driver*/
struct audio_dai_api;
struct audio_dai_cfg;

/* defined in the dmic driver*/
struct audio_dmic_api;
struct audio_dmic_cfg;

/* define in the dsp driver*/
struct audio_dsp_api;
struct audio_dsp_cfg;



/*
 * audio event that we support
 */
enum { ePLAY = 0,
       eRECORD,
       eLOOP,
       ePAUSE,
       eSTOP,
       eVOL,
       eMUTE,
} audio_event_t;

enum { eIDLE = 0,
       eBUSY,
} audio_status_t;

struct audio_device_par {
	struct audio_codec_cfg * a_codec_cfg;
	struct audio_dai_cfg * a_dai_cfg;
	struct audio_dmic_cfg * a_dmic_cfg;
	struct audio_dsp_cfg * a_dsp_cfg;
};

struct audio_device_api;

struct audio_device_info {
	sys_dnode_t node;
	char name[16];
	const struct audio_device_api *ops;
	struct device *dev;
};

struct audio_device_api {
	int (*init)(struct audio_device_info *dev,
		    struct audio_device_par *param);
	int (*config)(struct audio_device_info *dev,
		      struct audio_device_par *param);
	int (*send_event)(struct audio_device_info *dev, u32_t event_enum,
			  struct audio_device_par *param);
	int (*deinit)(struct audio_device_info *dev,
		      struct audio_device_par *param);
}

struct audio_events_ops;

struct audio_system {
	char name[16];
	const struct audio_events_ops *ops;
	sys_dlist_t device_list;
	u32_t status;
	struct k_mutex mutex;
};

struct audio_events_ops {
	int (*config)(struct audio_system *pas, struct audio_device_par *config);
	int (*send_event)(struct audio_system *, u32_t event_enum, audio_device_par *param);
	int (*polling_state)(struct audio_system *, u32_t *pevent_enum);
};

int audio_subsys_create_instance(const char *name, struct audio_system *paudio);

void audio_subsys_destroy_instance(struct audio_system *paudio,
				   bool deinit_device);

void audio_subsys_register_device(struct audio_device_info *device_info,
				  int type);

int audio_device_create_instance(struct device *dev,
				 struct audio_device_info **device_info);

void audio_subsys_regist_event_ops(struct audio_system *pas);

void audio_device_regist_event_ops(struct audio_device_info *pad);

int audio_system_suspend(struct audio_system * pas);

int audio_system_resume(struct audio_system * pas);

int audio_subsys_startup(struct audio_system *paudio);

int audio_subsys_shutdown(struct audio_system *paudio)

#ifdef __cplusplus
}
#endif

#endif