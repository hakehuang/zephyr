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

#include <audio/audio_core.h>

#if CONFIG_AUDIO_PM
	extern int aduio_machine_level_suspend(struct audio_system * pas);
	extern int aduio_machine_level_resume(struct audio_system * pas);
	extern int aduio_machine_level_startup(struct audio_system * pas);
	extern int aduio_machine_level_shutdown(struct audio_system * pas);
#endif
/*
 * suspend audio system
 */
int audio_system_suspend(struct audio_system * pas) {
	#if CONFIG_AUDIO_PM
		return aduio_machine_level_suspend(pas);
	#else
		return 0;
	#endif
}

/*
 * resume audio system
 */
int audio_system_resume(struct audio_system * pas) {
	#if CONFIG_AUDIO_PM
		return aduio_machine_level_resume(pas);
	#else
		return 0;
	#endif
}

/*
 *	audio_start - start audio device
 *  
 *	start audio deivces that register
 *
 *	Return: None
 */
int audio_subsys_startup(struct audio_system *paudio)
{
	#if CONFIG_AUDIO_PM
		return audio_machine_level_startup(paudio);
	#else
		return 0;
	#endif
}

/*
 *	aduio_shut_down - shutdown audio device
 *  
 *	start audio deivces that register
 *
 *	Return: None
 */
int aduio_subsys_shutdown(struct audio_system *paudio)
{
	#if CONFIG_AUDIO_PM
		return audio_machine_level_shutdown(paudio);
	#else
		return 0;
	#endif
}