/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * mira220 register definitions.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* mira220 registers */
#define MIRA220_REG_SENSOR_ID_L         0x102B
#define MIRA220_REG_SENSOR_ID_H         0x102C

#define MIRA220_REG_TEST_PATTERN        0x2091
#define MIRA220_REG_DELAY               0X7000

#define MIRA220_REG_SW_RESET            0x0040
/* Exposure time is indicated in number of rows */
#define MIRA220_REG_EXPOSURE            0x100C
#define MIRA220_REG_VBLANK              0x1012
/* Horizontal flip */
#define MIRA220_REG_HFLIP               0x209C
/* Vertical flip */
#define MIRA220_REG_VFLIP               0x1095
/* OTP control */
#define MIRA220_REG_OTP_CMD             0x0080
#define MIRA220_OTP_CMD_UP              (0x4)
#define MIRA220_OTP_CMD_DOWN            (0x8)
/* Image state register */
#define MIRA220_REG_IMAGER_STATE        0x1003
/* Image Start acquisition */
#define MIRA220_REG_IMAGER_RUN          0x10F0
/* Continuous running, not limited to nr of frames. */
#define MIRA220_IMAGER_RUN_CONT_REG     0x1002
#define MIRA220_IMAGER_RUN_CONT_ENABLE  (0x04)
#define MIRA220_IMAGER_RUN_CONT_DISABLE (0x00)

#ifdef __cplusplus
}
#endif
