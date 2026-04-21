/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "lrob001_settings.h"
#include "lrob001.h"

#define LROB001_IO_MUX_LOCK(mux)
#define LROB001_IO_MUX_UNLOCK(mux)
#define LROB001_ENABLE_OUT_XCLK(pin, clk)
#define LROB001_DISABLE_OUT_XCLK(pin)
#define LROB001_DEBUG_EN (0)

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms) vTaskDelay((ms > portTICK_PERIOD_MS ? ms / portTICK_PERIOD_MS : 1))

static const char *TAG = "lrob001";

#ifndef CONFIG_CAMERA_LROB001_MIPI_IF_FORMAT_INDEX_DEFAULT
#error "Please choose at least one format in menuconfig for LROB001"
#endif

#if !CONFIG_CAMERA_LROB001_MIPI_RAW8_1280X720_60FPS
#error "Please enable at least one format in menuconfig for LROB001"
#endif

/* Line timing placeholders for ISP; replace with values from sensor bring-up / tuning. */
static const esp_cam_sensor_isp_info_t lrob001_isp_info[] = {
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 160000000,
            .vts = 750,
            .hts = 2000,
            .tline_ns = 12500,
            .gain_def = 0,
            .exp_def = 0x100,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
};

static const uint8_t lrob001_format_default_index = CONFIG_CAMERA_LROB001_MIPI_IF_FORMAT_INDEX_DEFAULT;

static const uint8_t lrob001_format_index[] = {
#if CONFIG_CAMERA_LROB001_MIPI_RAW8_1280X720_60FPS
    0,
#endif
};

static const esp_cam_sensor_format_t lrob001_format_info[] = {
#if CONFIG_CAMERA_LROB001_MIPI_RAW8_1280X720_60FPS
    {
        .name = "MIPI_2lane_24Minput_RAW8_1280x720_60fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW8,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1280,
        .height = 720,
        .regs = lrob001_mipi_2lane_24Minput_1280x720_raw8_60fps,
        .regs_size = ARRAY_SIZE(lrob001_mipi_2lane_24Minput_1280x720_raw8_60fps),
        .fps = 60,
        .isp_info = &lrob001_isp_info[0],
        .mipi_info = {
            .mipi_clk = 400000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
#endif
};

static uint8_t get_lrob001_actual_format_index(void)
{
    for (int i = 0; i < ARRAY_SIZE(lrob001_format_index); i++) {
        if (lrob001_format_index[i] == lrob001_format_default_index) {
            return i;
        }
    }

    return 0;
}

static esp_err_t lrob001_read(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t *read_buf)
{
    return esp_sccb_transmit_receive_reg_a16v8(sccb_handle, reg, read_buf);
}

static esp_err_t lrob001_write(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t data)
{
    return esp_sccb_transmit_reg_a16v8(sccb_handle, reg, data);
}

static esp_err_t lrob001_read_block(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t *read_buf, size_t data_len)
{
    return esp_sccb_transmit_receive_block_a16(sccb_handle, reg, read_buf, data_len);
}

static esp_err_t lrob001_write_block(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t *data, size_t data_len)
{
    return esp_sccb_transmit_block_a16(sccb_handle, reg, data, data_len);
}

static esp_err_t lrob001_write_array(esp_sccb_io_handle_t sccb_handle, lrob001_reginfo_t *regarray, size_t regs_size)
{
    int i = 0;
    esp_err_t ret = ESP_OK;
    while ((ret == ESP_OK) && (i < regs_size)) {
        if (regarray[i].reg != LROB001_REG_DELAY) {
            ret = lrob001_write(sccb_handle, regarray[i].reg, regarray[i].val);
        } else {
            delay_ms(regarray[i].val);
        }
        i++;
    }
    ESP_LOGD(TAG, "Set array done[i=%d]", i);
    return ret;
}

static esp_err_t lrob001_hw_reset(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
    }
    return ESP_OK;
}

static esp_err_t lrob001_soft_reset(esp_cam_sensor_device_t *dev)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t lrob001_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    esp_err_t ret;
    uint8_t pid_h, pid_l;

    ret = lrob001_read(dev->sccb_handle, LROB001_REG_CHIP_ID_H, &pid_h);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = lrob001_read(dev->sccb_handle, LROB001_REG_CHIP_ID_L, &pid_l);
    if (ret != ESP_OK) {
        return ret;
    }
    id->pid = (pid_h << 8) | pid_l;

    return ret;
}

static esp_err_t lrob001_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    esp_err_t ret = lrob001_write(dev->sccb_handle, LROB001_REG_STREAM_CTRL, enable ? 0x01 : 0x00);
    if (ret == ESP_OK) {
        dev->stream_status = enable;
    }
    ESP_LOGW(TAG, "Stream=%d", enable);
    return ret;
}

static esp_err_t lrob001_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    ESP_LOGW(TAG, "query para is not supported");
    return ESP_OK;
}

static esp_err_t lrob001_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    ESP_LOGW(TAG, "get para is not supported");
    return ESP_OK;
}

static esp_err_t lrob001_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t lrob001_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);

    formats->count = ARRAY_SIZE(lrob001_format_info);
    formats->format_array = &lrob001_format_info[0];
    return ESP_OK;
}

static esp_err_t lrob001_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *sensor_cap)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, sensor_cap);

    sensor_cap->fmt_raw = 1;
    return ESP_OK;
}

static esp_err_t lrob001_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);

    esp_err_t ret = ESP_OK;
    if (format == NULL) {
        format = &lrob001_format_info[get_lrob001_actual_format_index()];
    }

    ret = lrob001_write_array(dev->sccb_handle, (lrob001_reginfo_t *)format->regs, format->regs_size);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set format regs fail");
        return ESP_CAM_SENSOR_ERR_FAILED_SET_FORMAT;
    }

    dev->cur_format = format;

    return ret;
}

static esp_err_t lrob001_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);

    esp_err_t ret = ESP_FAIL;

    if (dev->cur_format != NULL) {
        memcpy(format, dev->cur_format, sizeof(esp_cam_sensor_format_t));
        ret = ESP_OK;
    }
    return ret;
}

static esp_err_t lrob001_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    esp_err_t ret = ESP_OK;
    uint8_t regval;
    esp_cam_sensor_reg_val_t *sensor_reg;
    LROB001_IO_MUX_LOCK(mux);

    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_HW_RESET:
        ret = lrob001_hw_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_SW_RESET:
        ret = lrob001_soft_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_S_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = lrob001_write(dev->sccb_handle, sensor_reg->regaddr, sensor_reg->value);
        break;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        ret = lrob001_set_stream(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_G_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = lrob001_read(dev->sccb_handle, sensor_reg->regaddr, &regval);
        if (ret == ESP_OK) {
            sensor_reg->value = regval;
        }
        break;
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        ret = lrob001_get_sensor_id(dev, arg);
        break;
    default:
        break;
    }

    LROB001_IO_MUX_UNLOCK(mux);
    return ret;
}

static esp_err_t lrob001_power_on(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        LROB001_ENABLE_OUT_XCLK(dev->xclk_pin, dev->xclk_freq_hz);
        delay_ms(6);
    }

    if (dev->pwdn_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->pwdn_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        ret = gpio_config(&conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "pwdn pin config failed");
            return ret;
        }

        gpio_set_level(dev->pwdn_pin, 1);
        delay_ms(10);
        gpio_set_level(dev->pwdn_pin, 0);
        delay_ms(10);
    }

    if (dev->reset_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->reset_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        ret = gpio_config(&conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "reset pin config failed");
            return ret;
        }

        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
    }

    return ret;
}

static esp_err_t lrob001_power_off(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        LROB001_DISABLE_OUT_XCLK(dev->xclk_pin);
    }

    if (dev->pwdn_pin >= 0) {
        gpio_set_level(dev->pwdn_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->pwdn_pin, 1);
        delay_ms(10);
    }

    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
    }

    return ret;
}

static esp_err_t lrob001_delete(esp_cam_sensor_device_t *dev)
{
    ESP_LOGD(TAG, "del lrob001 (%p)", dev);
    if (dev) {
        lrob001_power_off(dev);
        free(dev);
        dev = NULL;
    }

    return ESP_OK;
}

static const esp_cam_sensor_ops_t lrob001_ops = {
    .query_para_desc = lrob001_query_para_desc,
    .get_para_value = lrob001_get_para_value,
    .set_para_value = lrob001_set_para_value,
    .query_support_formats = lrob001_query_support_formats,
    .query_support_capability = lrob001_query_support_capability,
    .set_format = lrob001_set_format,
    .get_format = lrob001_get_format,
    .priv_ioctl = lrob001_priv_ioctl,
    .del = lrob001_delete
};

esp_cam_sensor_device_t *lrob001_detect(esp_cam_sensor_config_t *config)
{
    esp_cam_sensor_device_t *dev = NULL;
    if (config == NULL) {
        return NULL;
    }

    dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "No memory for camera");
        return NULL;
    }

    dev->name = (char *)LROB001_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &lrob001_ops;
    dev->cur_format = &lrob001_format_info[get_lrob001_actual_format_index()];

    if (lrob001_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Camera power on failed");
        goto err_free_handler;
    }

    if (lrob001_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(TAG, "Get sensor ID failed");
        goto err_free_handler;
    } else if (dev->id.pid != LROB001_PID) {
        ESP_LOGE(TAG, "Camera sensor is not LROB001, PID=0x%x", dev->id.pid);
        goto err_free_handler;
    }
    ESP_LOGI(TAG, "Detected Camera sensor PID=0x%x", dev->id.pid);

    return dev;

err_free_handler:
    lrob001_power_off(dev);
    free(dev);

    return NULL;
}

#if CONFIG_CAMERA_LROB001_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(lrob001_detect, ESP_CAM_SENSOR_MIPI_CSI, LROB001_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return lrob001_detect(config);
}
#endif
