# SPDX-License-Identifier: Apache-2.0
zephyr_library()

zephyr_library_sources_ifdef(CONFIG_NXP_RT_AUDIO_SYS  nxp_rt_dapm.c)


zephyr_library_link_libraries(subsys__soc_audio)