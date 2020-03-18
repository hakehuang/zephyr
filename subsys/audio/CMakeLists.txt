# SPDX-License-Identifier: Apache-2.0

add_library(subsys__soc_audio INTERFACE)

target_include_directories(subsys__soc_audio INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(common)
add_subdirectory(machine)

zephyr_library_sources_ifdef(CONFIG_SOC_AUDIO  audio_core.c)
zephyr_library_sources_ifdef(CONFIG_SOC_AUDIO  audio_util.h)
zephyr_library_sources_ifdef(CONFIG_SOC_AUDIO  audio_dai.c)
zephyr_library_sources_ifdef(CONFIG_SOC_AUDIO  audio_damp.h)

