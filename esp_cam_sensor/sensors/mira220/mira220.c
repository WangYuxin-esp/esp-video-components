/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
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
#include "mira220_settings.h"
#include "mira220.h"

/* Mira220 does not support analog gain. */

typedef struct {
    uint32_t exposure_val;
    uint32_t exposure_max;
    uint32_t gain_index; // current gain index

    uint32_t vflip_en : 1;
    uint32_t hmirror_en : 1;
} mira220_para_t;

struct mira220_cam {
    mira220_para_t mira220_para;
};

#define MIRA220_IO_MUX_LOCK(mux)
#define MIRA220_IO_MUX_UNLOCK(mux)
#define MIRA220_ENABLE_OUT_XCLK(pin,clk)
#define MIRA220_DISABLE_OUT_XCLK(pin)

#define EXPOSURE_V4L2_UNIT_US                   100
#define EXPOSURE_V4L2_TO_mira220(v, sf)          \
    ((uint32_t)(((double)v) * (sf)->fps * (sf)->isp_info->isp_v1_info.vts / (1000000 / EXPOSURE_V4L2_UNIT_US) + 0.5))
#define EXPOSURE_MIRA220_TO_V4L2(v, sf)          \
    ((int32_t)(((double)v) * 1000000 / (sf)->fps / (sf)->isp_info->isp_v1_info.vts / EXPOSURE_V4L2_UNIT_US + 0.5))

#define MIRA220_VTS_MAX          0x7fff
#define MIRA220_EXP_MAX_OFFSET   0x06

#define MIRA220_PID         0x0130

// sensor name must same as cfg's sensor name
#define MIRA220_SENSOR_NAME "mira220"
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms)  vTaskDelay((ms > portTICK_PERIOD_MS ? ms/ portTICK_PERIOD_MS : 1))
#define MIRA220_SUPPORT_NUM CONFIG_CAMERA_MIRA220_MAX_SUPPORT

static size_t s_limited_gain_index;
static const uint8_t s_mira220_exp_min = 0x08;
static const char *TAG = "mira220";

static const esp_cam_sensor_isp_info_t mira220_isp_info[] = {
    /* For MIPI */
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 25190400,
            .vts = 600,  // 600 + 340
            .hts = 1024,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 25190400,
            .vts = 600,  // 600 + 340
            .hts = 1024,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    }
};

static const esp_cam_sensor_format_t mira220_format_info[] = {
    /* For MIPI */
    {
        .name = "MIPI_2lane_RAW12_1600_1400_25fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW12,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 38400000,
        .width = 1024,
        .height = 600,
        .regs = init_reglist_MIPI_2lane_1600_1400_25fps,
        .regs_size = ARRAY_SIZE(init_reglist_MIPI_2lane_1600_1400_25fps),
        .fps = 6,
        .isp_info = &mira220_isp_info[0],
        .mipi_info = {
            .mipi_clk = 200000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_RAW12_1280_720_25fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW8,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 38400000,
        .width = 1280,
        .height = 720,
        .regs = init_reglist_MIPI_2lane_1280_720_25fps,
        .regs_size = ARRAY_SIZE(init_reglist_MIPI_2lane_1280_720_25fps),
        .fps = 25,
        .isp_info = &mira220_isp_info[1],
        .mipi_info = {
            .mipi_clk = 200000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    }
};

static esp_err_t mira220_read(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t *read_buf)
{
    return esp_sccb_transmit_receive_reg_a16v8(sccb_handle, reg, read_buf);
}

static esp_err_t mira220_write(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t data)
{
    return esp_sccb_transmit_reg_a16v8(sccb_handle, reg, data);
}

/* write a array of registers  */
static esp_err_t mira220_write_array(esp_sccb_io_handle_t sccb_handle, mira220_reginfo_t *regarray, size_t regs_size)
{
    int i = 0;
    esp_err_t ret = ESP_OK;
    while ((ret == ESP_OK) && (i < regs_size)) {
        if (regarray[i].reg != MIRA220_REG_DELAY) {
            ret = mira220_write(sccb_handle, regarray[i].reg, regarray[i].val);
        } else {
            delay_ms(regarray[i].val);
        }
        i++;
    }
    ESP_LOGW(TAG, "Set array done[i=%d]", i);
    return ret;
}

static esp_err_t mira220_set_reg_bits(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t offset, uint8_t length, uint8_t value)
{
    esp_err_t ret = ESP_OK;
    uint8_t reg_data = 0;

    ret = mira220_read(sccb_handle, reg, &reg_data);
    if (ret != ESP_OK) {
        return ret;
    }
    uint8_t mask = ((1 << length) - 1) << offset;
    value = (reg_data & ~mask) | ((value << offset) & mask);
    ret = mira220_write(sccb_handle, reg, value);
    return ret;
}

static esp_err_t mira220_set_test_pattern(esp_cam_sensor_device_t *dev, int enable)
{
    return mira220_set_reg_bits(dev->sccb_handle, MIRA220_REG_TEST_PATTERN, 0, 1, enable ? 0x01 : 0x00);
}

static esp_err_t mira220_hw_reset(esp_cam_sensor_device_t *dev)
{
    return ESP_OK;
}

static esp_err_t mira220_soft_reset(esp_cam_sensor_device_t *dev)
{
    return mira220_write(dev->sccb_handle, MIRA220_REG_SW_RESET, 0x01);
}

static esp_err_t mira220_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{

    esp_err_t ret = ESP_FAIL;
    uint8_t pid_h, pid_l;

    ret = mira220_read(dev->sccb_handle, MIRA220_REG_SENSOR_ID_H, &pid_h);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mira220_read(dev->sccb_handle, MIRA220_REG_SENSOR_ID_L, &pid_l);
    if (ret != ESP_OK) {
        return ret;
    }
    id->pid = (pid_h << 8) | pid_l;
    ESP_LOGI(TAG, "MIRA220 READ 0x%x = 0x%x", MIRA220_REG_SENSOR_ID_H, pid_h);
    ESP_LOGI(TAG, "MIRA220 READ 0x%x = 0x%x", MIRA220_REG_SENSOR_ID_L, pid_l);

    return ESP_OK;
}

static esp_err_t mira220_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    esp_err_t ret = ESP_OK;
    ret = mira220_write(dev->sccb_handle, MIRA220_REG_IMAGER_STATE, enable ? 0x10 : 0x02);
    // delay_ms(10);

    ret |= mira220_write(dev->sccb_handle, MIRA220_REG_IMAGER_RUN, enable ? 0x01 : 0x00);
    // delay_ms(10);

    if (ret == ESP_OK) {
        dev->stream_status = enable;
    }
    ESP_LOGD(TAG, "Stream=%d", enable);
    return ret;
}

static esp_err_t mira220_set_mirror(esp_cam_sensor_device_t *dev, int enable)
{
    return mira220_write(dev->sccb_handle, MIRA220_REG_HFLIP, enable ? 0x01 : 0x00);
}

static esp_err_t mira220_set_vflip(esp_cam_sensor_device_t *dev, int enable)
{
    return mira220_write(dev->sccb_handle, MIRA220_REG_VFLIP, enable ? 0x01 : 0x00);
}

static esp_err_t mira220_set_exp_val(esp_cam_sensor_device_t *dev, uint32_t u32_val)
{
    esp_err_t ret;
    struct mira220_cam *cam_mira220 = (struct mira220_cam *)dev->priv;
    // Returns the maximum exposure time in row_length (reg value).
    // Calculation is baded on Mira220 datasheet Section 9.2.
    // max_exposure_time =  (height + vblank) - (int)(MIRA220_GLOB_NUM_CLK_CYCLES / row_length);

    uint32_t value_buf = MAX(u32_val, s_mira220_exp_min);
    value_buf = MIN(value_buf, cam_mira220->mira220_para.exposure_max);

    ESP_LOGD(TAG, "set exposure 0x%" PRIx32, value_buf);

    ret = mira220_write(dev->sccb_handle, MIRA220_REG_EXPOSURE, value_buf);

    if (ret == ESP_OK) {
        cam_mira220->mira220_para.exposure_val = value_buf;
    }
    return ret;
}

static esp_err_t mira220_set_total_gain_val(esp_cam_sensor_device_t *dev, uint32_t u32_val)
{
    return ESP_OK;
}

static esp_err_t mira220_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    esp_err_t ret = ESP_OK;
    return ret;

    // switch (qdesc->id) {
    // case ESP_CAM_SENSOR_EXPOSURE_VAL:
    //     qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
    //     qdesc->number.minimum = s_mira220_exp_min;
    //     qdesc->number.maximum = dev->cur_format->isp_info->isp_v1_info.vts - mira220_EXP_MAX_OFFSET; // max = VTS-6 = height+vblank-6, so when update vblank, exposure_max must be updated
    //     qdesc->number.step = 1;
    //     qdesc->default_value = dev->cur_format->isp_info->isp_v1_info.exp_def;
    //     break;
    // case ESP_CAM_SENSOR_EXPOSURE_US:
    //     qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
    //     qdesc->number.minimum = EXPOSURE_mira220_TO_V4L2(s_mira220_exp_min, dev->cur_format);
    //     qdesc->number.maximum = EXPOSURE_mira220_TO_V4L2((dev->cur_format->isp_info->isp_v1_info.vts - mira220_EXP_MAX_OFFSET), dev->cur_format); // max = VTS-6 = height+vblank-6, so when update vblank, exposure_max must be updated
    //     qdesc->number.step = EXPOSURE_mira220_TO_V4L2(0x01, dev->cur_format);
    //     qdesc->default_value = EXPOSURE_mira220_TO_V4L2((dev->cur_format->isp_info->isp_v1_info.exp_def), dev->cur_format);
    //     break;
    // case ESP_CAM_SENSOR_GAIN:
    //     qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_ENUMERATION;
    //     qdesc->enumeration.count = s_limited_gain_index;
    //     //qdesc->enumeration.elements = mira220_total_gain_val_map;
    //     qdesc->default_value = dev->cur_format->isp_info->isp_v1_info.gain_def; // gain index
    //     break;
    // case ESP_CAM_SENSOR_GROUP_EXP_GAIN:
    //     qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_U8;
    //     qdesc->u8.size = sizeof(esp_cam_sensor_gh_exp_gain_t);
    //     break;
    // case ESP_CAM_SENSOR_VFLIP:
    // case ESP_CAM_SENSOR_HMIRROR:
    //     qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
    //     qdesc->number.minimum = 0;
    //     qdesc->number.maximum = 1;
    //     qdesc->number.step = 1;
    //     qdesc->default_value = 0;
    //     break;
    // default: {
    //     ESP_LOGD(TAG, "id=%"PRIx32" is not supported", qdesc->id);
    //     ret = ESP_ERR_INVALID_ARG;
    //     break;
    // }
    // }
}

static esp_err_t mira220_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;
    return ret;

    struct mira220_cam *cam_mira220 = (struct mira220_cam *)dev->priv;
    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL: {
        *(uint32_t *)arg = cam_mira220->mira220_para.exposure_val;
        break;
    }
    case ESP_CAM_SENSOR_GAIN: {
        *(uint32_t *)arg = cam_mira220->mira220_para.gain_index;
        break;
    }
    default: {
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }
    }
}

static esp_err_t mira220_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;

    // switch (id) {
    // case ESP_CAM_SENSOR_EXPOSURE_VAL: {
    //     uint32_t u32_val = *(uint32_t *)arg;
    //     ret = mira220_set_exp_val(dev, u32_val);
    //     break;
    // }
    // case ESP_CAM_SENSOR_EXPOSURE_US: {
    //     uint32_t u32_val = *(uint32_t *)arg;
    //     uint32_t ori_exp = EXPOSURE_V4L2_TO_mira220(u32_val, dev->cur_format);
    //     ret = mira220_set_exp_val(dev, ori_exp);
    //     break;
    // }
    // case ESP_CAM_SENSOR_GAIN: {
    //     uint32_t u32_val = *(uint32_t *)arg;
    //     //ret = mira220_set_total_gain_val(dev, u32_val);
    //     break;
    // }
    // case ESP_CAM_SENSOR_GROUP_EXP_GAIN: {
    //     esp_cam_sensor_gh_exp_gain_t *value = (esp_cam_sensor_gh_exp_gain_t *)arg;
    //     uint32_t ori_exp = EXPOSURE_V4L2_TO_mira220(value->exposure_us, dev->cur_format);
    //     ret = mira220_write(dev->sccb_handle, MIRA220_REG_GROUP_HOLD, mira220_GROUP_HOLD_START);
    //     ret |= mira220_set_exp_val(dev, ori_exp);
    //     //ret |= mira220_set_total_gain_val(dev, value->gain_index);
    //     ret |= mira220_write(dev->sccb_handle, MIRA220_REG_GROUP_HOLD_DELAY, mira220_GROUP_HOLD_DELAY_FRAMES);
    //     ret |= mira220_write(dev->sccb_handle, MIRA220_REG_GROUP_HOLD, mira220_GROUP_HOLD_END);
    //     break;
    // }
    // case ESP_CAM_SENSOR_VFLIP: {
    //     int *value = (int *)arg;
    //     ret = mira220_set_vflip(dev, *value);
    //     break;
    // }
    // case ESP_CAM_SENSOR_HMIRROR: {
    //     int *value = (int *)arg;
    //     ret = mira220_set_mirror(dev, *value);
    //     break;
    // }
    // default: {
    //     ESP_LOGE(TAG, "set id=%" PRIx32 " is not supported", id);
    //     ret = ESP_ERR_INVALID_ARG;
    //     break;
    // }
    // }

    return ret;
}

static esp_err_t mira220_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);

    formats->count = ARRAY_SIZE(mira220_format_info);
    formats->format_array = &mira220_format_info[0];
    return ESP_OK;
}

static esp_err_t mira220_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *sensor_cap)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, sensor_cap);

    sensor_cap->fmt_raw = 1;
    return 0;
}

static esp_err_t mira220_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    struct mira220_cam *cam_mira220 = (struct mira220_cam *)dev->priv;
    esp_err_t ret = ESP_OK;

    if (format == NULL) {
        format = &mira220_format_info[CONFIG_CAMERA_MIRA220_MIPI_IF_FORMAT_INDEX_DEFAULT];
    }
    ret = mira220_write_array(dev->sccb_handle, (mira220_reginfo_t *)format->regs, format->regs_size);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set format regs fail");
        return ESP_CAM_SENSOR_ERR_FAILED_SET_FORMAT;
    }

    dev->cur_format = format;
    // // init para
    cam_mira220->mira220_para.exposure_val = dev->cur_format->isp_info->isp_v1_info.exp_def;
    // cam_mira220->mira220_para.gain_index = dev->cur_format->isp_info->isp_v1_info.gain_def;
    // cam_mira220->mira220_para.exposure_max = dev->cur_format->isp_info->isp_v1_info.vts - mira220_EXP_MAX_OFFSET;

    return ret;
}

static esp_err_t mira220_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
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

static esp_err_t mira220_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    esp_err_t ret = ESP_OK;
    uint8_t regval;
    ESP_LOGE(TAG, "mira220_priv_ioctl");

    esp_cam_sensor_reg_val_t *sensor_reg;
    MIRA220_IO_MUX_LOCK(mux);

    switch (cmd) {
    //case ESP_CAM_SENSOR_IOC_HW_RESET:
    //ret = mira220_hw_reset(dev);
    //break;
    //case ESP_CAM_SENSOR_IOC_SW_RESET:
    //ret = mira220_soft_reset(dev);
    //break;
    //case ESP_CAM_SENSOR_IOC_S_REG:
    //sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
    //ret = mira220_write(dev->sccb_handle, sensor_reg->regaddr, sensor_reg->value);
    //break;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        ret = mira220_set_stream(dev, *(int *)arg);
        ESP_LOGW(TAG, "SET STREAM");
        break;
    //case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
    //ret = mira220_set_test_pattern(dev, *(int *)arg);
    //break;
    //case ESP_CAM_SENSOR_IOC_G_REG:
    //sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
    //ret = mira220_read(dev->sccb_handle, sensor_reg->regaddr, &regval);
    //if (ret == ESP_OK) {
    //    sensor_reg->value = regval;
    //}
    //break;
    //case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
    //ret = mira220_get_sensor_id(dev, arg);
    //break;
    default:
        break;
    }

    MIRA220_IO_MUX_UNLOCK(mux);
    return ret;
}

static esp_err_t mira220_power_on(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        MIRA220_ENABLE_OUT_XCLK(dev->xclk_pin, dev->xclk_freq_hz);
    }

    if (dev->pwdn_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->pwdn_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        // carefully, logic is inverted compared to reset pin
        gpio_set_level(dev->pwdn_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->pwdn_pin, 1);
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
    ESP_LOGW(TAG, "PEDRO MIRA POWER ON");
    return ret;
}

static esp_err_t mira220_power_off(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        MIRA220_DISABLE_OUT_XCLK(dev->xclk_pin);
    }

    if (dev->pwdn_pin >= 0) {
        gpio_set_level(dev->pwdn_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->pwdn_pin, 1);
        delay_ms(10);
    }

    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
    }
    return ret;
}

static esp_err_t mira220_delete(esp_cam_sensor_device_t *dev)
{
    if (dev) {
        if (dev->priv) {
            free(dev->priv);
            dev->priv = NULL;
        }
        free(dev);
        dev = NULL;
    }

    return ESP_OK;
}

static const esp_cam_sensor_ops_t mira220_ops = {
    .query_para_desc = mira220_query_para_desc,
    .get_para_value = mira220_get_para_value,
    .set_para_value = mira220_set_para_value,
    .query_support_formats = mira220_query_support_formats,
    .query_support_capability = mira220_query_support_capability,
    .set_format = mira220_set_format,
    .get_format = mira220_get_format,
    .priv_ioctl = mira220_priv_ioctl,
    .del = mira220_delete
};

esp_cam_sensor_device_t *mira220_detect(esp_cam_sensor_config_t *config)
{
    esp_cam_sensor_device_t *dev = NULL;
    struct mira220_cam *cam_mira220;

    // s_limited_gain_index = ARRAY_SIZE(mira220_total_gain_val_map);
    // if (config == NULL) {
    //     return NULL;
    // }

    dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (dev == NULL) {
        return NULL;
    }

    cam_mira220 = heap_caps_calloc(1, sizeof(struct mira220_cam), MALLOC_CAP_DEFAULT);
    if (!cam_mira220) {
        free(dev);
        return NULL;
    }

    dev->name = (char *)MIRA220_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &mira220_ops;
    dev->priv = cam_mira220;
    dev->cur_format = &mira220_format_info[CONFIG_CAMERA_MIRA220_MIPI_IF_FORMAT_INDEX_DEFAULT];

    // for (size_t i = 0; i < ARRAY_SIZE(mira220_total_gain_val_map); i++) {
    //     if (mira220_total_gain_val_map[i] > s_limited_gain) {
    //         s_limited_gain_index = i - 1;
    //         break;
    //     }
    // }

    // Configure sensor power, clock, and SCCB port
    if (mira220_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Camera power on failed");
        goto err_free_handler;
    }

    if (mira220_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(TAG, "Get sensor ID failed");
        goto err_free_handler;
    } else if (dev->id.pid != MIRA220_PID) {
        ESP_LOGE(TAG, "Camera sensor is not mira220, PID=0x%x", dev->id.pid);
        goto err_free_handler;
    }

    return dev;

err_free_handler:
    mira220_power_off(dev);
    free(dev->priv);
    free(dev);

    return NULL;
}

#if CONFIG_CAMERA_MIRA220_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(mira220_detect, ESP_CAM_SENSOR_MIPI_CSI, MIRA220_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return mira220_detect(config);
}
#endif
