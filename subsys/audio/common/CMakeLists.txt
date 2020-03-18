# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SOC_AUDIO  audio_common.c)
zephyr_library_sources_ifdef(CONFIG_SOC_AUDIO  audio_common.h)

zephyr_library_link_libraries(subsys__soc_audio)