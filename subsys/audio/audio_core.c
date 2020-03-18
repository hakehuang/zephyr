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
#include <kernel.h>

#define LOG_LEVEL CONFIG_SOC_AUDIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(soc_audio);

#define SOUND_TYPE 16

/*
 * create a audio system instance in given name
 * @name: in, the audio system name
 * @ppaudio: out, the point to the audio system
 *
 * return:
 *   return 0 on SUCCESS, other -1
 */
int audio_subsys_create_instance(const char *name,
				 struct audio_system **ppaudio)
{
	struct audio_system *paudio = NULL;
	paudio = (struct audio_system *)k_malloc(sizeof(struct audio_system));

	if (NULL == paudio) {
		return -1;
	}

	if (NULL == name) {
		strcpy(paudio->name, "unnamed audio");
	}
	sys_dlist_init(&(paudio->device_list));
	paudio->status = eIDLE;
	k_mutex_init(&(paudio->mutex));
	audio_system_regist_event_ops(paudio);
	*ppaudio = paudio;

	return 0;
}

/*
 * destroy an audio system instance
 * paudio: in, audio subsys pointer
 * deinit_device: in, deinit device or not
 * 
 * Return: No
 */
void audio_subsys_destroy_instance(struct audio_system *paudio,
				   bool deinit_device)
{
	sys_dlist_t *pdlist = NULL;
	sys_dnode_t *pnode = NULL;
	sys_dnode_t *pnode_safe = NULL;
	struct audio_device_info *paudio_device = NULL;
	int ret = 0;
	u32_t ret_status = eBUSY;

	if (NULL == ppaudio)
		return;

	paudio->ops->send_event(paudio, eSTOP);

	while (ret_status != eIDLE) {
		paudio->ops->polling_state(paudio, &ret_status);
	}

	pdlist = &paudio->device_list;

	SYS_DLIST_FOR_EACH_CONTAINER_SAFE (pdlist, paudio_device,
					   paudio_device_safe, node) {
		if (deinit_device) {
			if (paudio_device->ops && paudio_device->ops->deinit) {
				ret = paudio_device->ops->deinit(
					paudio_device->dev, NULL);
				if (!ret) {
					LOG_WRN("deinit %s failure",
						paudio_device->name)
					//return;
				}
			}
		}
		k_free(paudio_device);
	}

	k_free(paudio);

	return;
}

/*
 *	audio_subsys_register_device - register a sound node
 *	@device_info: device card info
 *	@type: device type
 *  
 *	register a device type with name and set to source card list
 *
 *	Return: None
 */
void audio_subsys_register_device(struct audio_system *paudio,
				  struct audio_device_info *device_info,
				  int type)
{
	int rc = 0;
	const char *name;
	char _name[16];

	switch (type) {
	case 0:
		/*audio codec*/
		name = "codec";
		break;
	case 1:
		/*I2S, AC97, PCM, PDM, TDM interface*/
		name = "dai";
		break;
	case 2:
		/*dsp device controller*/
		name = "dsp";
		break;
	case 3:
		/*dmic peripheral*/
		name = "dmic";
		break;
	default: {
	__unknown:
		sprintf(_name, "unknown%d", chain);
		if (unit >= SOUND_STEP)
			strcat(_name, "-");
		name = _name;
	} break;
	}

	k_mutex_lock(&(paudio->mutex), K_FOREVER);

	strncpy(device_info->name, name, 16);
	sys_dlist_append(&audio_device_list, &device_info->node);
	LOG_DBG("audio card(%s) registred", disk->name);

	k_mutex_unlock(&(paudio->mutex));
	return;
}

/*
 * malloc audio device structure for given device pointer
 * @dev: in, device pointer
 * @device_info: out, created audio device info pointer
 * 
 * the create device will be freed in audio_subsys_destroy_instance
 *
 * Return: 0 on SUCCESS, otherwise -1
 */
int audio_device_create_instance(struct device *dev,
				 struct audio_device_info **device_info)
{
	struct audio_device_info *padi = NULL;
	pdi = (struct audio_device_info *)k_malloc(struct audio_device_info);
	if (NULL == padi) {
		LOG_WRN("fail to malloc for device");
		return -1;
	}
	padi->dev = dev;
	audio_device_regist_event_ops(padi);
	*device_info = padi;
	return 0;
}



