/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

#include "esp_xclk_generator.h"

static const char *TAG = "xclk";

#if CONFIG_LEDC_XCLK_ENABLE
/**
 * @brief XCLK generator instance type implemented by LEDC
 */
typedef struct xclk_generator_ledc_t {
    ledc_timer_t ledc_channel;
    struct esp_xclk_generator_t base;
} xclk_generator_ledc_t;

#define NO_CAMERA_LEDC_CHANNEL 0xFF
/*Note that when setting the resolution to 1, we can only choose to divide the frequency by 1 times from CLK*/
#define XCLK_LEDC_DUTY_RES_DEFAULT  LEDC_TIMER_1_BIT
#define XCLK_LEDC_MEM_CAPS          MALLOC_CAP_DEFAULT

static esp_err_t xclk_timer_conf(ledc_timer_t ledc_timer, ledc_clk_cfg_t clk_cfg, int xclk_freq_hz)
{
    esp_err_t err;
    ledc_timer_config_t timer_conf = {0};
    timer_conf.duty_resolution = XCLK_LEDC_DUTY_RES_DEFAULT;
    timer_conf.freq_hz = xclk_freq_hz;
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.clk_cfg = clk_cfg;
    timer_conf.timer_num = (ledc_timer_t)ledc_timer;
    err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed for freq %d, rc=%x", xclk_freq_hz, err);
    }
    return err;
}

/**
 * @brief Stop xclk output, and set idle level
 *
 * @param  ledc_channel LEDC channel (0-7), select from ledc_channel_t
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
static esp_err_t xclk_ledc_generator_stop(esp_xclk_generator_t *xclk_handle)
{
    esp_err_t ret = ESP_OK;
    xclk_generator_ledc_t *xclk_ledc = __containerof(xclk_handle, xclk_generator_ledc_t, base);

    if (xclk_ledc->ledc_channel != NO_CAMERA_LEDC_CHANNEL) {
        ret = ledc_stop(LEDC_LOW_SPEED_MODE, xclk_ledc->ledc_channel, 0);
    }
    heap_caps_free(xclk_ledc);
    return ret;
}

esp_err_t xclk_ledc_generator_start(ledc_timer_t ledc_timer, ledc_clk_cfg_t clk_cfg, ledc_channel_t ledc_channel, int xclk_freq_hz, int pin_xclk, esp_xclk_generator_handle_t *xclk_handle)
{
    ESP_RETURN_ON_FALSE(xclk_handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_err_t err = xclk_timer_conf(ledc_timer, clk_cfg, xclk_freq_hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
        return err;
    }
    xclk_generator_ledc_t *xclk_ledc = heap_caps_calloc(1, sizeof(xclk_generator_ledc_t), XCLK_LEDC_MEM_CAPS);
    ESP_RETURN_ON_FALSE(xclk_ledc, ESP_ERR_NO_MEM, TAG, "no mem for xclk handle");

    ledc_channel_config_t ch_conf = {0};
    ch_conf.gpio_num = pin_xclk;
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf.channel = ledc_channel;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.timer_sel = ledc_timer;
    ch_conf.duty = 1;
    ch_conf.hpoint = 0;
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        free(xclk_ledc);
        ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
        return err;
    }

    xclk_ledc->ledc_channel = ledc_channel;
    xclk_ledc->base.stop = xclk_ledc_generator_stop;
    *xclk_handle = &(xclk_ledc->base);
    ESP_LOGD(TAG, "new xclk ledc: %p", xclk_ledc);
    return ESP_OK;
}
#endif

esp_err_t esp_xclk_generator_stop(esp_xclk_generator_handle_t xclk_handle)
{
    ESP_RETURN_ON_FALSE(xclk_handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument: null pointer");
    ESP_RETURN_ON_FALSE(xclk_handle->stop, ESP_ERR_NOT_SUPPORTED, TAG, "driver not supported");

    return xclk_handle->stop(xclk_handle);
}
