/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdio.h>
#if CONFIG_LEDC_XCLK_ENABLE
#include "driver/ledc.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief xclk generator handle type
 */
typedef struct esp_xclk_generator_t *esp_xclk_generator_handle_t;

/**
 * @brief xclk generator controller type
 */
typedef struct esp_xclk_generator_t esp_xclk_generator_t;

/**
 * @brief xclk generator controller type
 */
struct esp_xclk_generator_t {
    /**
     * @brief Stop xclk output
     *
     * @param[in] handle xclk generator handle
     * @return
     *        - ESP_OK: Success.
     */
    esp_err_t (*stop)(esp_xclk_generator_t *xclk_handle);
};

/**
 * @brief Stop xclk output
 *
 * @param[in] handle xclk generator handle
 * @return
 *        - ESP_OK: Success.
 */
esp_err_t esp_xclk_generator_stop(esp_xclk_generator_handle_t xclk_handle);

#if CONFIG_LEDC_XCLK_ENABLE
/**
 * @brief Configure LEDC timer&channel for generating XCLK
 *        Configure LEDC timer with the given source timer/frequency(Hz)
 *
 * @param  ledc_timer The timer source of channel
 * @param  clk_cfg LEDC source clock from ledc_clk_cfg_t
 * @param  ledc_channel LEDC channel used for XCLK (0-7)
 * @param  xclk_freq_hz XCLK output frequency (Hz)
 * @param  pin_xclk the XCLK output gpio_num, if you want to use gpio16, gpio_num = 16
 * @param  xclk_handle xclk handle
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Can not find a proper pre-divider number base on the given frequency and the current duty_resolution.
 */
esp_err_t xclk_ledc_generator_start(ledc_timer_t ledc_timer, ledc_clk_cfg_t clk_cfg, ledc_channel_t ledc_channel, int xclk_freq_hz, int pin_xclk, esp_xclk_generator_handle_t *xclk_handle);
#endif

#ifdef __cplusplus
}
#endif
