# Copyright (c) 2025 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_MIXER app PRIVATE input_processor_mixer.c)
