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

/*
 * LROB001 register entry (16-bit address / 8-bit value), SCCB format a16v8.
 */
typedef struct {
    uint16_t reg;
    uint8_t val;
} lrob001_reginfo_t;

#ifdef __cplusplus
}
#endif
