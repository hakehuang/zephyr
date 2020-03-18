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

/*
 * machine level suspend
 */
int aduio_machine_level_suspend(struct audio_system * pas) {
	return 0;
}

/*
 * machine level resume
 */
int aduio_machine_level_resume(struct audio_system * pas) {
	return 0;
}

/*
 * machine level startup
 */
int audio_machine_startup(struct audio_system * pas) {
	return 0;
}

/*
 * machine level shutdown
 */
int audio_machine_shutdown(struct audio_system * pas) {
	return 0;
}



