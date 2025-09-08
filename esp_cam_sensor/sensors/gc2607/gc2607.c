/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
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
#include "gc2607_settings.h"
#include "gc2607.h"

/*
 * GC2607 camera sensor gain control.
 */
typedef struct {
    uint8_t again_fine; // analog gain fine
    uint8_t again_coarse; // analog gain coarse
    uint8_t dgain_fine; // digital gain fine
    uint8_t dgain_coarse; // digital gain coarse
} gc2607_gain_t;

typedef struct {
    uint32_t exposure_val;
    uint32_t exposure_max;
    uint32_t gain_index; // current gain index

    uint32_t vflip_en : 1;
    uint32_t hmirror_en : 1;
} gc2607_para_t;

struct gc2607_cam {
    gc2607_para_t gc2607_para;
};

#define GC2607_IO_MUX_LOCK(mux)
#define GC2607_IO_MUX_UNLOCK(mux)
#define GC2607_ENABLE_OUT_XCLK(pin,clk)
#define GC2607_DISABLE_OUT_XCLK(pin)

/*
* Min_Frame_len = window height + 80
* If Exposure < Min_Frame_len, Actual frame len = Min_Frame_len; If Exposure > Min_Frame_len, Actual frame len = exposure + 16;
* Line_exp_in_s = HTS * Time_of_PCLK
* PCLK = HTS * VTS * fps
* Exposure_step = 1/16 * Line_exp_in_s = 1/16 * HTS * Time_of_PCLK = 1/16 * HTS * 1/PCLK
* Exposure_step = 1/16 * HTS * 1/(HTS * VTS * fps) = 1/16 * 1/(VTS * fps)
*/
#define EXPOSURE_V4L2_UNIT_US                   100
#define EXPOSURE_V4L2_TO_GC2607(v, sf)          \
    ((uint32_t)(((double)v) * (sf)->fps * (sf)->isp_info->isp_v1_info.vts * 16 / (1000000 / EXPOSURE_V4L2_UNIT_US) + 0.5))
#define EXPOSURE_GC2607_TO_V4L2(v, sf)          \
    ((int32_t)(((double)v) * 1000000 / (sf)->fps / (sf)->isp_info->isp_v1_info.vts / 16 / EXPOSURE_V4L2_UNIT_US + 0.5))

#define GC2607_FETCH_EXP_H(val)     (((val) >> 8) & 0x3F)
#define GC2607_FETCH_EXP_L(val)     ((val) & 0xFF)
#define GC2607_GROUP_HOLD_START   0x00
#define GC2607_GROUP_HOLD_LUNCH   0x30
#define GC2607_EXP_MAX_OFFSET     0x06

#define GC2607_PID         0x2607
#define GC2607_SENSOR_NAME "GC2607"
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms)  vTaskDelay((ms > portTICK_PERIOD_MS ? ms/ portTICK_PERIOD_MS : 1))
#define GC2607_SUPPORT_NUM CONFIG_CAMERA_GC2607_MAX_SUPPORT

static const uint8_t s_gc2607_exp_min = 0x10;
static const char *TAG = "GC2607";

// total gain = analog_gain x digital_gain x 1000(To avoid decimal points, the final abs_gain is multiplied by 1000.)
static const uint32_t gc2607_total_gain_val_map[] = {
    //1x
    1000,
};

// GC2607 Gain map format: [ANG_FINE(0x3e09), ANG_COARSE(0x3e08), DIG_FINE(0x3e07), DIG_COARSE(0x3e06)]
static const gc2607_gain_t gc2607_gain_map[] = {
    // 1x
    {0x10, 0x00, 0x80, 0x00},
};

#if CONFIG_SOC_MIPI_CSI_SUPPORTED
static const esp_cam_sensor_isp_info_t gc2607_isp_info_mipi[] = {
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 84000000,
            .vts = 1367,
            .hts = 2048,
            .gain_def = 0,
            .exp_def = 0x0438,
            .bayer_type = ESP_CAM_SENSOR_BAYER_GRBG,
        }
    },
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 84000000,
            .vts = 1640,
            .hts = 2048,
            .gain_def = 0,
            .exp_def = 0x0438,
            .bayer_type = ESP_CAM_SENSOR_BAYER_GRBG,
        }
    },
    {
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 84000000,
            .vts = 1367,
            .hts = 2048,
            .gain_def = 0,
            .exp_def = 0x0438,
            .bayer_type = ESP_CAM_SENSOR_BAYER_GRBG,
        }
    },
};

static const esp_cam_sensor_format_t gc2607_format_info_mipi[] = {
    {
        .name = "MIPI_2lane_24Minput_raw10_960x540_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 960,
        .height = 540,
        .regs = mipi_24Minput_2lane_960x540_raw10_30fps,
        .regs_size = ARRAY_SIZE(mipi_24Minput_2lane_960x540_raw10_30fps),
        .fps = 30,
        .isp_info = &gc2607_isp_info_mipi[0],
        .mipi_info = {
            .mipi_clk = 336000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_24Minput_raw10_1920x1080_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1920,
        .height = 1080,
        .regs = mipi_24Minput_2lane_1920x1080_raw10_30fps,
        .regs_size = ARRAY_SIZE(mipi_24Minput_2lane_1920x1080_raw10_30fps),
        .fps = 30,
        .isp_info = &gc2607_isp_info_mipi[1],
        .mipi_info = {
            .mipi_clk = 672000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_24Minput_raw10_960x540_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 960,
        .height = 540,
        .regs = mipi_24Minput_2lane_960x540_raw10_29fps,
        .regs_size = ARRAY_SIZE(mipi_24Minput_2lane_960x540_raw10_29fps),
        .fps = 30,
        .isp_info = &gc2607_isp_info_mipi[2],
        .mipi_info = {
            .mipi_clk = 336000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    }
};
#endif

static esp_err_t gc2607_read(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t *read_buf)
{
    return esp_sccb_transmit_receive_reg_a16v8(sccb_handle, reg, read_buf);
}

static esp_err_t gc2607_write(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t data)
{
    return esp_sccb_transmit_reg_a16v8(sccb_handle, reg, data);
}

static esp_err_t gc2607_set_reg_bits(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t offset, uint8_t length, uint8_t value)
{
    esp_err_t ret = ESP_OK;
    uint8_t reg_data = 0;

    ret = gc2607_read(sccb_handle, reg, &reg_data);
    if (ret != ESP_OK) {
        return ret;
    }
    uint8_t mask = ((1 << length) - 1) << offset;
    value = (reg_data & ~mask) | ((value << offset) & mask);
    ret = gc2607_write(sccb_handle, reg, value);
    return ret;
}

/* write a array of registers  */
static esp_err_t gc2607_write_array(esp_sccb_io_handle_t sccb_handle, gc2607_reginfo_t *regarray, size_t regs_size)
{
    int i = 0;
    esp_err_t ret = ESP_OK;
    while ((ret == ESP_OK) && (i < regs_size)) {
        if (regarray[i].reg != GC2607_REG_DELAY) {
            ret = gc2607_write(sccb_handle, regarray[i].reg, regarray[i].val);
        } else {
            delay_ms(regarray[i].val);
        }
        i++;
    }
    ESP_LOGD(TAG, "Set array done[i=%d]", i);
    return ret;
}

static esp_err_t gc2607_set_test_pattern(esp_cam_sensor_device_t *dev, int enable)
{
    return gc2607_set_reg_bits(dev->sccb_handle, 0x008c, 2, 1, enable ? 0x01 : 0x00);
}

static esp_err_t gc2607_hw_reset(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
    }
    return ESP_OK;
}

static esp_err_t gc2607_soft_reset(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = gc2607_set_reg_bits(dev->sccb_handle, 0x0103, 0, 1, 0x01);
    delay_ms(5);
    return ret;
}

static esp_err_t gc2607_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t pid_h, pid_l;

    ret = gc2607_read(dev->sccb_handle, GC2607_REG_ID_HIGH, &pid_h);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = gc2607_read(dev->sccb_handle, GC2607_REG_ID_LOW, &pid_l);
    if (ret != ESP_OK) {
        return ret;
    }
    id->pid = (pid_h << 8) | pid_l;

    return ret;
}

static esp_err_t gc2607_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
#if 0
    esp_err_t ret = ESP_FAIL;

    ret = gc2607_write(dev->sccb_handle, GC2607_REG_SLEEP_MODE, enable ? 0x01 : 0x00);
    if (enable) {
        ret |= gc2607_write(dev->sccb_handle, 0x4418, 0x0a);
        ret |= gc2607_write(dev->sccb_handle, 0x363d, 0x10);
        ret |= gc2607_write(dev->sccb_handle, 0x4419, 0x80);
    }
#endif
    dev->stream_status = enable;
    ESP_LOGD(TAG, "Stream=%d", enable);

    return ESP_OK;
}

static esp_err_t gc2607_set_mirror(esp_cam_sensor_device_t *dev, int enable)
{
    return gc2607_set_reg_bits(dev->sccb_handle, 0x0101, 0, 1, enable ? 0x01 : 0x00);
}

static esp_err_t gc2607_set_vflip(esp_cam_sensor_device_t *dev, int enable)
{
    return gc2607_set_reg_bits(dev->sccb_handle, 0x0101, 1, 1, enable ? 0x01 : 0x00);
}

static esp_err_t gc2607_set_exp_val(esp_cam_sensor_device_t *dev, uint32_t u32_val)
{
    esp_err_t ret;
    struct gc2607_cam *cam_gc2607 = (struct gc2607_cam *)dev->priv;
    uint32_t value_buf = MAX(u32_val, s_gc2607_exp_min);
    value_buf = MIN(value_buf, cam_gc2607->gc2607_para.exposure_max);

    ESP_LOGD(TAG, "set exposure 0x%" PRIx32, value_buf);

    ret = gc2607_write(dev->sccb_handle, GC2607_REG_SHUTTER_TIME_H, GC2607_FETCH_EXP_H(u32_val));
    ret |= gc2607_write(dev->sccb_handle, GC2607_REG_SHUTTER_TIME_L, GC2607_FETCH_EXP_L(u32_val));
    if (ret == ESP_OK) {
        cam_gc2607->gc2607_para.exposure_val = value_buf;
    }
    return ret;
}

static esp_err_t gc2607_set_total_gain_val(esp_cam_sensor_device_t *dev, uint32_t u32_val)
{
#if 0
    esp_err_t ret;
    struct gc2607_cam *cam_gc2607 = (struct gc2607_cam *)dev->priv;

    ESP_LOGD(TAG, "again_fine %" PRIx8 ", again_coarse %" PRIx8 ", dgain_fine %" PRIx8 ", dgain_coarse %" PRIx8, gc2607_gain_map[u32_val].again_fine,
             gc2607_gain_map[u32_val].again_coarse,
             gc2607_gain_map[u32_val].dgain_fine,
             gc2607_gain_map[u32_val].dgain_coarse);

    ret = gc2607_write(dev->sccb_handle,
                       GC2607_REG_FINE_AGAIN,
                       gc2607_gain_map[u32_val].again_fine);

    ret |= gc2607_set_reg_bits(dev->sccb_handle,
                               GC2607_REG_COARSE_AGAIN, 2, 3,
                               gc2607_gain_map[u32_val].again_coarse);

    ret |= gc2607_write(dev->sccb_handle,
                        GC2607_REG_FINE_DGAIN,
                        gc2607_gain_map[u32_val].dgain_fine);

    ret |= gc2607_set_reg_bits(dev->sccb_handle,
                               GC2607_REG_COARSE_DGAIN, 0, 2,
                               gc2607_gain_map[u32_val].dgain_coarse);
    if (ret == ESP_OK) {
        cam_gc2607->gc2607_para.gain_index = u32_val;
    }
    return ret;
#endif
    return ESP_OK;
}

static esp_err_t gc2607_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    esp_err_t ret = ESP_OK;
    switch (qdesc->id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = s_gc2607_exp_min;
        qdesc->number.maximum = dev->cur_format->isp_info->isp_v1_info.vts - GC2607_EXP_MAX_OFFSET; // max = VTS-6 = height+vblank-6, so when update vblank, exposure_max must be updated
        qdesc->number.step = 1;
        qdesc->default_value = dev->cur_format->isp_info->isp_v1_info.exp_def;
        break;
    case ESP_CAM_SENSOR_EXPOSURE_US:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = MAX(0x01, EXPOSURE_GC2607_TO_V4L2(s_gc2607_exp_min, dev->cur_format)); // The minimum value must be greater than 1
        qdesc->number.maximum = EXPOSURE_GC2607_TO_V4L2((dev->cur_format->isp_info->isp_v1_info.vts - GC2607_EXP_MAX_OFFSET), dev->cur_format);
        qdesc->number.step = MAX(0x01, EXPOSURE_GC2607_TO_V4L2(0x01, dev->cur_format));
        qdesc->default_value = EXPOSURE_GC2607_TO_V4L2((dev->cur_format->isp_info->isp_v1_info.exp_def), dev->cur_format);
        break;
    case ESP_CAM_SENSOR_GAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_ENUMERATION;
        qdesc->enumeration.count = ARRAY_SIZE(gc2607_total_gain_val_map);
        qdesc->enumeration.elements = gc2607_total_gain_val_map;
        qdesc->default_value = dev->cur_format->isp_info->isp_v1_info.gain_def; // gain index
        break;
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_U8;
        qdesc->u8.size = sizeof(esp_cam_sensor_gh_exp_gain_t);
        break;
    case ESP_CAM_SENSOR_VFLIP:
    case ESP_CAM_SENSOR_HMIRROR:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = 0;
        qdesc->number.maximum = 1;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;
    default: {
        ESP_LOGD(TAG, "id=%"PRIx32" is not supported", qdesc->id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    }
    return ret;
}

static esp_err_t gc2607_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;
    struct gc2607_cam *cam_gc2607 = (struct gc2607_cam *)dev->priv;
    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL: {
        *(uint32_t *)arg = cam_gc2607->gc2607_para.exposure_val;
        break;
    }
    case ESP_CAM_SENSOR_GAIN: {
        *(uint32_t *)arg = cam_gc2607->gc2607_para.gain_index;
        break;
    }
    default: {
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }
    }
    return ret;
}

static esp_err_t gc2607_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;

    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL: {
        uint32_t u32_val = *(uint32_t *)arg;
        ESP_LOGD(TAG, "set exposure 0x%" PRIx32, u32_val);
        ret = gc2607_set_exp_val(dev, u32_val);
        break;
    }
    case ESP_CAM_SENSOR_EXPOSURE_US: {
        uint32_t u32_val = *(uint32_t *)arg;
        uint32_t ori_exp = EXPOSURE_V4L2_TO_GC2607(u32_val, dev->cur_format);
        ret = gc2607_set_exp_val(dev, ori_exp);
        break;
    }
    case ESP_CAM_SENSOR_GAIN: {
        uint32_t u32_val = *(uint32_t *)arg;
        ret = gc2607_set_total_gain_val(dev, u32_val);
        break;
    }
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN: {
        esp_cam_sensor_gh_exp_gain_t *value = (esp_cam_sensor_gh_exp_gain_t *)arg;
        uint32_t ori_exp = EXPOSURE_V4L2_TO_GC2607(value->exposure_us, dev->cur_format);
        ret = gc2607_set_exp_val(dev, ori_exp);
        ret |= gc2607_set_total_gain_val(dev, value->gain_index);
        break;
    }
    case ESP_CAM_SENSOR_VFLIP: {
        int *value = (int *)arg;

        ret = gc2607_set_vflip(dev, *value);
        break;
    }
    case ESP_CAM_SENSOR_HMIRROR: {
        int *value = (int *)arg;

        ret = gc2607_set_mirror(dev, *value);
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

static esp_err_t gc2607_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);
#if CONFIG_SOC_MIPI_CSI_SUPPORTED
    if (dev->sensor_port == ESP_CAM_SENSOR_MIPI_CSI) {
        formats->count = ARRAY_SIZE(gc2607_format_info_mipi);
        formats->format_array = &gc2607_format_info_mipi[0];
    }
#endif

    return ESP_OK;
}

static esp_err_t gc2607_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *sensor_cap)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, sensor_cap);

    sensor_cap->fmt_yuv = 1;
    return 0;
}

static esp_err_t gc2607_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);

    esp_err_t ret = ESP_OK;
    struct gc2607_cam *cam_gc2607 = (struct gc2607_cam *)dev->priv;
    /* Depending on the interface type, an available configuration is automatically loaded.
    You can set the output format of the sensor without using query_format().*/
    if (format == NULL) {
#if CONFIG_SOC_MIPI_CSI_SUPPORTED
        if (dev->sensor_port == ESP_CAM_SENSOR_MIPI_CSI) {
            format = &gc2607_format_info_mipi[CONFIG_CAMERA_GC2607_MIPI_IF_FORMAT_INDEX_DEFAULT];
        }
#endif
    }

    ret = gc2607_write_array(dev->sccb_handle, (gc2607_reginfo_t *)format->regs, format->regs_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set format regs fail");
        return ESP_CAM_SENSOR_ERR_FAILED_SET_FORMAT;
    }
    ESP_LOGD(TAG, "Set format %s", format->name);
    dev->cur_format = format;
    // init para
    cam_gc2607->gc2607_para.exposure_val = dev->cur_format->isp_info->isp_v1_info.exp_def;
    cam_gc2607->gc2607_para.gain_index = dev->cur_format->isp_info->isp_v1_info.gain_def;
    cam_gc2607->gc2607_para.exposure_max = dev->cur_format->isp_info->isp_v1_info.vts - GC2607_EXP_MAX_OFFSET;

    return ret;
}

static esp_err_t gc2607_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
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

static esp_err_t gc2607_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    esp_err_t ret = ESP_OK;
    uint8_t regval;
    esp_cam_sensor_reg_val_t *sensor_reg;
    GC2607_IO_MUX_LOCK(mux);

    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_HW_RESET:
        ret = gc2607_hw_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_SW_RESET:
        ret = gc2607_soft_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_S_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = gc2607_write(dev->sccb_handle, sensor_reg->regaddr, sensor_reg->value);
        break;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        ret = gc2607_set_stream(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
        ret = gc2607_set_test_pattern(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_G_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = gc2607_read(dev->sccb_handle, sensor_reg->regaddr, &regval);
        if (ret == ESP_OK) {
            sensor_reg->value = regval;
        }
        break;
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        ret = gc2607_get_sensor_id(dev, arg);
        break;
    default:
        break;
    }

    GC2607_IO_MUX_UNLOCK(mux);
    return ret;
}

static esp_err_t gc2607_power_on(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        GC2607_ENABLE_OUT_XCLK(dev->xclk_pin, dev->xclk_freq_hz);
        delay_ms(2);
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

static esp_err_t gc2607_power_off(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        GC2607_DISABLE_OUT_XCLK(dev->xclk_pin);
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

static esp_err_t gc2607_delete(esp_cam_sensor_device_t *dev)
{
    ESP_LOGD(TAG, "del gc2607 (%p)", dev);
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

static const esp_cam_sensor_ops_t gc2607_ops = {
    .query_para_desc = gc2607_query_para_desc,
    .get_para_value = gc2607_get_para_value,
    .set_para_value = gc2607_set_para_value,
    .query_support_formats = gc2607_query_support_formats,
    .query_support_capability = gc2607_query_support_capability,
    .set_format = gc2607_set_format,
    .get_format = gc2607_get_format,
    .priv_ioctl = gc2607_priv_ioctl,
    .del = gc2607_delete
};

esp_cam_sensor_device_t *gc2607_detect(esp_cam_sensor_config_t *config)
{
    esp_cam_sensor_device_t *dev = NULL;
    struct gc2607_cam *cam_gc2607;

    if (config == NULL) {
        return NULL;
    }

    dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "No memory for camera");
        return NULL;
    }

    cam_gc2607 = heap_caps_calloc(1, sizeof(struct gc2607_cam), MALLOC_CAP_DEFAULT);
    if (!cam_gc2607) {
        ESP_LOGE(TAG, "failed to calloc cam");
        free(dev);
        return NULL;
    }

    dev->name = (char *)GC2607_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &gc2607_ops;
    dev->priv = cam_gc2607;
#if CONFIG_SOC_MIPI_CSI_SUPPORTED
    if (config->sensor_port == ESP_CAM_SENSOR_MIPI_CSI) {
        dev->cur_format = &gc2607_format_info_mipi[CONFIG_CAMERA_GC2607_MIPI_IF_FORMAT_INDEX_DEFAULT];
    }
#endif

    // Configure sensor power, clock, and SCCB port
    if (gc2607_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Camera power on failed");
        goto err_free_handler;
    }

    if (gc2607_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(TAG, "Get sensor ID failed");
        goto err_free_handler;
    } else if (dev->id.pid != GC2607_PID) {
        ESP_LOGE(TAG, "Camera sensor is not GC2607, PID=0x%x", dev->id.pid);
        goto err_free_handler;
    }
    ESP_LOGI(TAG, "Detected Camera sensor PID=0x%x", dev->id.pid);

    return dev;

err_free_handler:
    gc2607_power_off(dev);
    free(dev->priv);
    free(dev);

    return NULL;
}

#if CONFIG_CAMERA_GC2607_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(gc2607_detect, ESP_CAM_SENSOR_MIPI_CSI, GC2607_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return gc2607_detect(config);
}
#endif
