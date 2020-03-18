/*
 * Copyright (c) 2020 NXP Semiconductor INC.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <string.h>
#include <zephyr/types.h>
#include <string.h>
#include <version.h>

#include <soc.h>
#include <toolchain.h>
#include <errno.h>
#include <sys/atomic.h>
#include <sys/dlist.h>

#include <audio/audio_core.h>

#include <common/audio_common.h>

#define LOG_LEVEL CONFIG_SOC_AUDIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(soc_audio);

/*
 * config all devices in this subsys
 * @pas: in, audio system
 * @config: in, config parameters
 *
 * Return: 0 on success, -1 otherwise
 */
static int _audio_subsys_config(struct audio_system *pas,
				struct audio_device_par *config)
{
	sys_dlist_t *pdlist = NULL;
	sys_dnode_t *pnode = NULL;
	sys_dnode_t *pnode_safe = NULL;

	SYS_DLIST_FOR_EACH_CONTAINER_SAFE (pdlist, paudio_device,
					   paudio_device_safe, node) {
		if (paudio_device->ops && paudio_device->ops->config) {
			int ret = 0;
			ret = paudio_device->ops->config(paudio_device->device,
							 config);
			if (!ret) {
				LOG_WRN("config device %s error",
					paudio_device->name);
			}
		}
	}
	return 0;
}

/*
 * send event message all devices in this subsys
 * @pas: in, audio system
 * @event_enum: event type
 * @param: in, event parameters
 *
 * Return: 0 on success, -1 otherwise
 */
static int _audio_subsys_send_event(struct audio_system *pas, u32_t event_enum,
				    struct audio_device_par *param)
{
	sys_dlist_t *pdlist = NULL;
	sys_dnode_t *pnode = NULL;
	sys_dnode_t *pnode_safe = NULL;

	SYS_DLIST_FOR_EACH_CONTAINER_SAFE (pdlist, paudio_device,
					   paudio_device_safe, node) {
		if (paudio_device->ops && paudio_device->ops->send_event) {
			int ret = 0;
			ret = paudio_device->ops->send_event(
				paudio_device->device, event_enum, param);
			if (!ret) {
				LOG_WRN("config send_event %s error",
					paudio_device->name);
				return -1;
			}
		}
	}
	return 0;
}

/*
 * polling subsys status
 * @pas: in, pointer to audio system
 * @pevent_enum: out, status
 *
 * return: No
 */
static void _audio_subsys_polling_state(struct audio_system *pas,
					u32_t *pevent_enum)
{
	*pevent_enum = pas->status;
}

static struct audio_events_ops audio_event_ops = {
	.config = _audio_subsys_config,
	.send_event = _audio_subsys_send_event,
	.polling_state = _audio_subsys_polling_state,
}


/*
 * register operation for event
 */
void audio_subsys_regist_event_ops(struct audio_system * pas)
{
	pas->audio_system_regist_event_ops = audio_event_ops;
}

/*
 * default init device
 * @dev: in, device pointer
 * @param: in init param
 * 
 * Return: 0 on success, otherwise fail
 */
static int _device_init(struct audio_device_info *p_ad_info,
			struct audio_device_par *param)
{
	if (NULL != p_ad_info->dev->config &&
	    NULL != p_ad_info->dev->config->init) {
		return p_ad_info->dev->config->init(dev);
	}
	return 0;
}

/*
 * default config device
 * @dev: in, device pointer
 * @param: in init param
 * 
 * Return: 0 on success, otherwise fail
 */
static int _device_config(struct audio_device_info *p_ad_info,
			  struct audio_device_par *param)
{
	if (NULL != p_ad_info->dev->driver_api) {
		if (strcmp(p_ad_info->name, "codec") == 0) {
			struct audio_codec_api *ac_api =
				(struct audio_codec_api *)
					p_ad_info->dev->driver_api;
			if (NULL == param->a_codec_cfg) {
				/*no need to config this device*/
				return 0;
			} else {
				return ac_api->configure(
					p_ad_info->dev,
					(struct audio_codec_cfg *)
						param->a_codec_cfg);
			}
		} else if (strcmp(p_ad_info->name, "dai") == 0) {
			struct audio_dai_api *ac_api =
				(struct audio_dai_api *)
					p_ad_info->dev->driver_api;
			if (NULL == param->a_dai_cfg) {
				/*no need to config this device*/
				return 0;
			} else {
				return ac_api->configure(
					p_ad_info->dev,
					(struct audio_dai_cfg *)
						param->a_dai_cfg);
			}
		} else if (strcmp(p_ad_info->name, "dsp") == 0) {
			struct audio_dsp_api *ac_api =
				(struct audio_dsp_api *)
					p_ad_info->dev->driver_api;
			if (param->a_dsp_cfg == NULL) {
				return 0;
			} else {
				return ac_api->configure(
					p_ad_info->dev,
					(struct audio_dsp_cfg *)
						param->a_dsp_cfg);
			}
		} else if (strcmp(p_ad_info->name, "dmic") == 0) {
			struct audio_dmic_api *ac_api =
				(struct audio_dmic_api *)
					p_ad_info->dev->driver_api;
			if (param->a_dmic_cfg == NULL) {
				return ac_api->configure(
					p_ad_info->dev,
					(struct audio_dmic_cfg *)
						param->a_dmic_cfg);
			}
		} else {
			LOG_WRN("no a current support audio device type %s",
				p_ad_info->name);
		}
	}
	return 0;
}

/*
 * default event parser for device
 * @dev: in, device pointer
 * @event_enum: in, event type defined in audio_event_t
 * @param: in init param
 * 
 * Return: 0 on success, otherwise fail
 */
static int _device_send_event(struct audio_device_info *p_ad_info,
			      u32_t event_enum, struct audio_device_par *param)
{
	int ret = 0;
	switch (event_enum) {
	case ePLAY:
		ret = audio_playback(p_ad_info, param);
		break;
	case eRECORD:
		ret = audio_record(p_ad_info, param);
		break;
	case eLOOP:
		ret = audio_loopback(p_ad_info, param);
		break;
	case ePAUSE:
		ret = audio_pause(p_ad_info, param);
		break;
	case eSTOP:
		ret = audio_stop(p_ad_info, param);
		break;
	case eVOL:
		ret = audio_vol(p_ad_info, param);
		break;
	case eMUTE:
		ret = audio_mute(p_ad_info, param);
		break;
	default:
		break;
	}
	return ret;
}

static int _device_deinit(struct audio_device_info *p_ad_info,
			  struct audio_device_par *param)
{
	return 0;
}

static struct _audio_device_api ad_apis = {
	.init = _device_init,
	.config = _device_config,
	.send_event = _device_send_event,
	.deinit = _device_deinit,
};

void audio_device_regist_event_ops(struct audio_device_info *pad)
{
	pad->ops = _audio_device_api;
}
