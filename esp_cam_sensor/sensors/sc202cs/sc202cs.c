/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "sc202cs_settings.h"
#include "sc202cs.h"

#define SC202CS_IO_MUX_LOCK(mux)
#define SC202CS_IO_MUX_UNLOCK(mux)
#define SC202CS_ENABLE_OUT_XCLK(pin,clk)
#define SC202CS_DISABLE_OUT_XCLK(pin)

#define SC202CS_PID         0xeb52
#define SC202CS_SENSOR_NAME "SC202CS"
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms)  vTaskDelay((ms > portTICK_PERIOD_MS ? ms/ portTICK_PERIOD_MS : 1))
#define SC202CS_SUPPORT_NUM CONFIG_CAMERA_SC202CS_MAX_SUPPORT

static const char *TAG = "sc202cs";

static const esp_cam_sensor_isp_info_t sc202cs_isp_info[] = {
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 72000000,
            .vts = 1250,
            .hts = 1920,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 72000000,
            .vts = 1250,
            .hts = 1920,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 72000000,
            .vts = 1250,
            .hts = 1920,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 72000000,
            .vts = 1250,
            .hts = 1920,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
};

static const esp_cam_sensor_format_t sc202cs_format_info[] = {
    {
        .name = "MIPI_1lane_24Minput_RAW8_1280x720_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW8,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1280,
        .height = 720,
        .regs = init_reglist_MIPI_1lane_raw8_1280x720_30fps,
        .regs_size = ARRAY_SIZE(init_reglist_MIPI_1lane_raw8_1280x720_30fps),
        .fps = 30,
        .isp_info = &sc202cs_isp_info[0],
        .mipi_info = {
            .mipi_clk = 576000000,
            .lane_num = 1,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_1lane_24Minput_RAW8_1600x1200_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW8,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1600,
        .height = 1200,
        .regs = init_reglist_MIPI_1lane_raw8_1600x1200_30fps,
        .regs_size = ARRAY_SIZE(init_reglist_MIPI_1lane_raw8_1600x1200_30fps),
        .fps = 30,
        .isp_info = &sc202cs_isp_info[1],
        .mipi_info = {
            .mipi_clk = 576000000,
            .lane_num = 1,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_1lane_24Minput_RAW10_1600x1200_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1600,
        .height = 1200,
        .regs = init_reglist_MIPI_1lane_raw10_1600x1200_30fps,
        .regs_size = ARRAY_SIZE(init_reglist_MIPI_1lane_raw10_1600x1200_30fps),
        .fps = 30,
        .isp_info = &sc202cs_isp_info[2],
        .mipi_info = {
            .mipi_clk = 720000000,
            .lane_num = 1,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_1lane_24Minput_RAW10_1600x900_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1600,
        .height = 900,
        .regs = init_reglist_MIPI_1lane_raw10_1600x900_30fps,
        .regs_size = ARRAY_SIZE(init_reglist_MIPI_1lane_raw10_1600x900_30fps),
        .fps = 30,
        .isp_info = &sc202cs_isp_info[3],
        .mipi_info = {
            .mipi_clk = 720000000,
            .lane_num = 1,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
};

static esp_err_t sc202cs_read(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t *read_buf)
{
    return esp_sccb_transmit_receive_reg_a16v8(sccb_handle, reg, read_buf);
}

static esp_err_t sc202cs_write(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t data)
{
    return esp_sccb_transmit_reg_a16v8(sccb_handle, reg, data);
}

/* write a array of registers  */
static esp_err_t sc202cs_write_array(esp_sccb_io_handle_t sccb_handle, sc202cs_reginfo_t *regarray)
{
    int i = 0;
    esp_err_t ret = ESP_OK;
    while ((ret == ESP_OK) && regarray[i].reg != SC202CS_REG_END) {
        if (regarray[i].reg != SC202CS_REG_DELAY) {
            ret = sc202cs_write(sccb_handle, regarray[i].reg, regarray[i].val);
        } else {
            delay_ms(regarray[i].val);
        }
        i++;
    }
    return ret;
}

static esp_err_t sc202cs_set_reg_bits(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t offset, uint8_t length, uint8_t value)
{
    esp_err_t ret = ESP_OK;
    uint8_t reg_data = 0;

    ret = sc202cs_read(sccb_handle, reg, &reg_data);
    if (ret != ESP_OK) {
        return ret;
    }
    uint8_t mask = ((1 << length) - 1) << offset;
    value = (ret & ~mask) | ((value << offset) & mask);
    ret = sc202cs_write(sccb_handle, reg, value);
    return ret;
}

static esp_err_t sc202cs_set_test_pattern(esp_cam_sensor_device_t *dev, int enable)
{
    return sc202cs_set_reg_bits(dev->sccb_handle, 0x4501, 3, 1, enable ? 0x01 : 0x00);
}

static esp_err_t sc202cs_hw_reset(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
    }
    return ESP_OK;
}

static esp_err_t sc202cs_soft_reset(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = sc202cs_set_reg_bits(dev->sccb_handle, 0x0103, 0, 1, 0x01);
    delay_ms(5);
    return ret;
}

static esp_err_t sc202cs_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t pid_h, pid_l;

    ret = sc202cs_read(dev->sccb_handle, SC202CS_REG_SENSOR_ID_H, &pid_h);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = sc202cs_read(dev->sccb_handle, SC202CS_REG_SENSOR_ID_L, &pid_l);
    if (ret != ESP_OK) {
        return ret;
    }
    id->pid = (pid_h << 8) | pid_l;

    return ret;
}

static esp_err_t sc202cs_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    esp_err_t ret = ESP_FAIL;
    ret = sc202cs_write(dev->sccb_handle, SC202CS_REG_SLEEP_MODE, enable ? 0x01 : 0x00);

    dev->stream_status = enable;
    ESP_LOGD(TAG, "Stream=%d", enable);
    return ret;
}

static esp_err_t sc202cs_set_mirror(esp_cam_sensor_device_t *dev, int enable)
{
    return sc202cs_set_reg_bits(dev->sccb_handle, 0x3221, 1, 2, enable ? 0x03 : 0x00);
}

static esp_err_t sc202cs_set_vflip(esp_cam_sensor_device_t *dev, int enable)
{
    return sc202cs_set_reg_bits(dev->sccb_handle, 0x3221, 5, 2, enable ? 0x03 : 0x00);
}

static esp_err_t sc202cs_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t sc202cs_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t sc202cs_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;

    switch (id) {
    case ESP_CAM_SENSOR_VFLIP: {
        int *value = (int *)arg;

        ret = sc202cs_set_vflip(dev, *value);
        break;
    }
    case ESP_CAM_SENSOR_HMIRROR: {
        int *value = (int *)arg;

        ret = sc202cs_set_mirror(dev, *value);
        break;
    }
    default: {
        ESP_LOGE(TAG, "set id=%" PRIx32 " is not supported", id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    }

    return ret;
}

static esp_err_t sc202cs_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);

    formats->count = ARRAY_SIZE(sc202cs_format_info);
    formats->format_array = &sc202cs_format_info[0];
    return ESP_OK;
}

static esp_err_t sc202cs_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *sensor_cap)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, sensor_cap);

    sensor_cap->fmt_raw = 1;
    return 0;
}

static esp_err_t sc202cs_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);

    esp_err_t ret = ESP_OK;
    /* Depending on the interface type, an available configuration is automatically loaded.
    You can set the output format of the sensor without using query_format().*/
    if (format == NULL) {
        format = &sc202cs_format_info[CONFIG_CAMERA_SC202CS_MIPI_IF_FORMAT_INDEX_DAFAULT];
    }

    ret = sc202cs_write_array(dev->sccb_handle, (sc202cs_reginfo_t *)format->regs);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set format regs fail");
        return ESP_CAM_SENSOR_ERR_FAILED_SET_FORMAT;
    }

    dev->cur_format = format;

    return ret;
}

static esp_err_t sc202cs_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
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

static esp_err_t sc202cs_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    esp_err_t ret = ESP_OK;
    uint8_t regval;
    esp_cam_sensor_reg_val_t *sensor_reg;
    SC202CS_IO_MUX_LOCK(mux);

    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_HW_RESET:
        ret = sc202cs_hw_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_SW_RESET:
        ret = sc202cs_soft_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_S_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = sc202cs_write(dev->sccb_handle, sensor_reg->regaddr, sensor_reg->value);
        break;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        ret = sc202cs_set_stream(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
        ret = sc202cs_set_test_pattern(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_G_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = sc202cs_read(dev->sccb_handle, sensor_reg->regaddr, &regval);
        if (ret == ESP_OK) {
            sensor_reg->value = regval;
        }
        break;
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        ret = sc202cs_get_sensor_id(dev, arg);
        break;
    default:
        break;
    }

    SC202CS_IO_MUX_UNLOCK(mux);
    return ret;
}

static esp_err_t sc202cs_power_on(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        SC202CS_ENABLE_OUT_XCLK(dev->xclk_pin, dev->xclk_freq_hz);
    }

    if (dev->pwdn_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->pwdn_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        // carefully, logic is inverted compared to reset pin
        gpio_set_level(dev->pwdn_pin, 1);
        delay_ms(10);
        gpio_set_level(dev->pwdn_pin, 0);
        delay_ms(10);
    }

    if (dev->reset_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->reset_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
    }

    return ret;
}

static esp_err_t sc202cs_power_off(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        SC202CS_DISABLE_OUT_XCLK(dev->xclk_pin);
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

static esp_err_t sc202cs_delete(esp_cam_sensor_device_t *dev)
{
    ESP_LOGD(TAG, "del sc202cs (%p)", dev);
    if (dev) {
        free(dev);
        dev = NULL;
    }

    return ESP_OK;
}

static const esp_cam_sensor_ops_t sc202cs_ops = {
    .query_para_desc = sc202cs_query_para_desc,
    .get_para_value = sc202cs_get_para_value,
    .set_para_value = sc202cs_set_para_value,
    .query_support_formats = sc202cs_query_support_formats,
    .query_support_capability = sc202cs_query_support_capability,
    .set_format = sc202cs_set_format,
    .get_format = sc202cs_get_format,
    .priv_ioctl = sc202cs_priv_ioctl,
    .del = sc202cs_delete
};

esp_cam_sensor_device_t *sc202cs_detect(esp_cam_sensor_config_t *config)
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

    dev->name = (char *)SC202CS_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &sc202cs_ops;
    dev->cur_format = &sc202cs_format_info[CONFIG_CAMERA_SC202CS_MIPI_IF_FORMAT_INDEX_DAFAULT];

    // Configure sensor power, clock, and SCCB port
    if (sc202cs_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Camera power on failed");
        goto err_free_handler;
    }

    if (sc202cs_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(TAG, "Get sensor ID failed");
        goto err_free_handler;
    } else if (dev->id.pid != SC202CS_PID) {
        ESP_LOGE(TAG, "Camera sensor is not SC202CS, PID=0x%x", dev->id.pid);
        goto err_free_handler;
    }
    ESP_LOGI(TAG, "Detected Camera sensor PID=0x%x", dev->id.pid);

    return dev;

err_free_handler:
    sc202cs_power_off(dev);
    free(dev);

    return NULL;
}

#if CONFIG_CAMERA_SC202CS_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(sc202cs_detect, ESP_CAM_SENSOR_MIPI_CSI, SC202CS_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return sc202cs_detect(config);
}
#endif
