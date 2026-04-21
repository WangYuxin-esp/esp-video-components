/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_TYPE_WRITE       0
#define CFG_TYPE_DELAY       1
#define CFG_TYPE_BLOCK_WRITE 2

typedef struct {
    uint8_t  type;       /* 0=WRITE, 1=DELAY, 2=BLOCK_WRITE */
    uint16_t reg_addr;   /* 16-bit 寄存器地址 */
    uint8_t  reg_value;  /* 8-bit 寄存器值 */
    uint32_t delay_us;   /* 微秒延时 (仅 DELAY 类型有效) */
} lrob001_reginfo_t;

#ifdef __cplusplus
}
#endif
