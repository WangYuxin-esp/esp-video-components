/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdio.h>
#include <stdint.h>
#include <sdkconfig.h>
#include "gc4053_regs.h"
#include "gc4053_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_SOC_MIPI_CSI_SUPPORTED
#if CONFIG_CAMERA_GC4053_MIPI_RAW10_1920X1080_30FPS
#include "gc4053_mipi_2lane_24Minput_raw10_1920x1080_30fps.h"
#endif
#if CONFIG_CAMERA_GC4053_MIPI_RAW10_1440X1440_30FPS
#include "gc4053_mipi_2lane_24Minput_raw10_1440x1440_30fps.h"
#endif
#endif

#ifdef __cplusplus
}
#endif
