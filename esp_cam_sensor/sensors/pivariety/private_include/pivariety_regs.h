/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * pivariety register definitions.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define PIVARIETY_REG_END      0xffff
#define PIVARIETY_REG_DELAY    0xfffe

/* pivariety registers */
#define STREAM_ON                      0x0100
#define DEVICE_VERSION_REG             0x0101
#define SENSOR_ID_REG                  0x0102
#define DEVICE_ID_REG                  0x0103
#define FOCUS_MOTOR_REG                0x0104
#define FIRMWARE_SENSOR_ID_REG         0x0105
#define UNIQUE_ID_REG                  0x0106
#define SYSTEM_IDLE_REG                0x0107

/* pivariety pixformat registers */
#define PIXFORMAT_INDEX_REG            0x0200
#define PIXFORMAT_TYPE_REG             0x0201
#define BAYER_ORDER_REG                0x0202
#define MIPI_LANES_REG                 0x0203
#define FLIPS_DONT_CHANGE_ORDER_REG    0x0204

/* pivariety resolution registers */
#define RESOLUTION_INDEX_REG           0x0300
#define FORMAT_WIDTH_REG               0x0301
#define FORMAT_HEIGHT_REG              0x0302

/* pivariety control registers */
#define CTRL_INDEX_REG                 0x0400
#define CTRL_ID_REG                    0x0401
#define CTRL_MIN_REG                   0x0402
#define CTRL_MAX_REG                   0x0403
#define CTRL_STEP_REG                  0x0404
#define CTRL_DEF_REG                   0x0405
#define CTRL_VALUE_REG                 0x0406

#define SENSOR_RD_REG                  0x0501
#define SENSOR_WR_REG                  0x0502





#ifdef __cplusplus
}
#endif
