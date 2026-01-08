/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * pivariety camera sensor register type definition.
 */
typedef struct {
    uint16_t reg;
    uint32_t val;
} pivariety_reginfo_t;

#ifdef __cplusplus
}
#endif
