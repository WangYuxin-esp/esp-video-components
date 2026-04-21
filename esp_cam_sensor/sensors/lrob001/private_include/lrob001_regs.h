/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * LROB001 register definitions (defaults follow LT6911-style banked access; replace per datasheet).
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define LROB001_REG_DELAY               0xffff

#define LROB001_REG_STREAM_CTRL         0x0007

#define LROB001_REG_CHIP_ID_H           0x0000
#define LROB001_REG_CHIP_ID_L           0x0001

#ifdef __cplusplus
}
#endif
