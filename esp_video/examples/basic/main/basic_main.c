/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "example_video_common.h"

#define BUFFER_COUNT      2
#define FRAME_HEAD_BYTES  16

typedef enum {
    APP_VIDEO_FMT_RAW8 = V4L2_PIX_FMT_SBGGR8,
    APP_VIDEO_FMT_RAW10 = V4L2_PIX_FMT_SBGGR10,
    APP_VIDEO_FMT_GREY = V4L2_PIX_FMT_GREY,
    APP_VIDEO_FMT_RGB565 = V4L2_PIX_FMT_RGB565,
    APP_VIDEO_FMT_RGB565_BE = V4L2_PIX_FMT_RGB565X,
    APP_VIDEO_FMT_RGB888 = V4L2_PIX_FMT_RGB24,
    APP_VIDEO_FMT_YUV422_YUYV = V4L2_PIX_FMT_YUYV,
    APP_VIDEO_FMT_YUV422_UYVY = V4L2_PIX_FMT_UYVY,
    APP_VIDEO_FMT_YUV420 = V4L2_PIX_FMT_YUV420,
    APP_VIDEO_FMT_YUV444 = V4L2_PIX_FMT_YUV444,
} video_fmt_t;

static const char *TAG = "basic";

static void print_frame_head_hex(const uint8_t *data, uint32_t bytesused)
{
    char hex_str[FRAME_HEAD_BYTES * 3 + 1];
    uint32_t print_len = bytesused < FRAME_HEAD_BYTES ? bytesused : FRAME_HEAD_BYTES;
    char *p = hex_str;

    for (uint32_t i = 0; i < print_len; i++) {
        p += snprintf(p, sizeof(hex_str) - (p - hex_str), "%02x ", data[i]);
    }
    *p = '\0';

    ESP_LOGI(TAG, "frame head (%" PRIu32 " bytes): %s", print_len, hex_str);
}

static void print_fourcc(uint32_t fourcc, char *out)
{
    out[0] = (char)(fourcc & 0xff);
    out[1] = (char)((fourcc >> 8) & 0xff);
    out[2] = (char)((fourcc >> 16) & 0xff);
    out[3] = (char)((fourcc >> 24) & 0xff);
    out[4] = '\0';
}

static esp_err_t enum_and_print_formats(int fd, int type)
{
    ESP_LOGI(TAG, "Enumerating supported formats:");

    for (int fmt_index = 0; ; fmt_index++) {
        struct v4l2_fmtdesc fmtdesc = {
            .index = fmt_index,
            .type = type,
        };
        char fourcc[5];

        if (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != 0) {
            break;
        }

        print_fourcc(fmtdesc.pixelformat, fourcc);
        ESP_LOGI(TAG, "  [%d] pixelformat=0x%08" PRIx32 " (%s), description=%s, flags=0x%x",
                 fmt_index, fmtdesc.pixelformat, fourcc, fmtdesc.description, fmtdesc.flags);

        struct v4l2_frmsizeenum frmsize = {
            .index = 0,
            .pixel_format = fmtdesc.pixelformat,
            .type = type,
        };

        if (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
            for (int size_index = 0; ; size_index++) {
                frmsize.index = size_index;
                if (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) != 0) {
                    break;
                }
                ESP_LOGI(TAG, "       size[%d]: %" PRIu32 "x%" PRIu32,
                         size_index, frmsize.discrete.width, frmsize.discrete.height);
            }
        }
    }

    return ESP_OK;
}

static esp_err_t get_first_format(int fd, int type, uint32_t *pixelformat, uint32_t *width, uint32_t *height)
{
    struct v4l2_fmtdesc fmtdesc = {
        .index = 0,
        .type = type,
    };

    ESP_RETURN_ON_FALSE(ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0, ESP_ERR_NOT_FOUND, TAG,
                        "no supported format");

    *pixelformat = fmtdesc.pixelformat;

    struct v4l2_frmsizeenum frmsize = {
        .index = 0,
        .pixel_format = fmtdesc.pixelformat,
        .type = type,
    };

    if (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
        *width = frmsize.discrete.width;
        *height = frmsize.discrete.height;
    } else {
        struct v4l2_format format = {
            .type = type,
        };
        ESP_RETURN_ON_FALSE(ioctl(fd, VIDIOC_G_FMT, &format) == 0, ESP_FAIL, TAG,
                            "failed to get default format");
        *width = format.fmt.pix.width;
        *height = format.fmt.pix.height;
    }

    return ESP_OK;
}

static esp_err_t basic_capture_loop(int fd, video_fmt_t init_fmt)
{
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uint32_t pixelformat;
    uint32_t width;
    uint32_t height;
    uint8_t *buffer[BUFFER_COUNT];
    struct v4l2_buffer buf;
    struct v4l2_requestbuffers req;
    struct v4l2_format format;
    char fourcc[5];

    ESP_RETURN_ON_ERROR(get_first_format(fd, type, &pixelformat, &width, &height), TAG,
                        "get format failed");

    memset(&format, 0, sizeof(format));
    format.type = type;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = init_fmt;
    ESP_RETURN_ON_FALSE(ioctl(fd, VIDIOC_S_FMT, &format) == 0, ESP_FAIL, TAG,
                        "failed to set format");

    print_fourcc(pixelformat, fourcc);
    ESP_LOGI(TAG, "Set format: %s, %" PRIu32 "x%" PRIu32,
             fourcc, format.fmt.pix.width, format.fmt.pix.height);

    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;
    req.type = type;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_RETURN_ON_FALSE(ioctl(fd, VIDIOC_REQBUFS, &req) == 0, ESP_FAIL, TAG,
                        "failed to request buffers");

    for (int i = 0; i < BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        ESP_RETURN_ON_FALSE(ioctl(fd, VIDIOC_QUERYBUF, &buf) == 0, ESP_FAIL, TAG,
                            "failed to query buffer");

        buffer[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd, buf.m.offset);
        ESP_RETURN_ON_FALSE(buffer[i], ESP_ERR_NO_MEM, TAG, "failed to mmap buffer");

        ESP_RETURN_ON_FALSE(ioctl(fd, VIDIOC_QBUF, &buf) == 0, ESP_FAIL, TAG,
                            "failed to queue buffer");
    }

    ESP_RETURN_ON_FALSE(ioctl(fd, VIDIOC_STREAMON, &type) == 0, ESP_FAIL, TAG,
                        "failed to start stream");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = V4L2_MEMORY_MMAP;
        ESP_RETURN_ON_FALSE(ioctl(fd, VIDIOC_DQBUF, &buf) == 0, ESP_FAIL, TAG,
                            "failed to dequeue buffer");

        if (buf.flags & V4L2_BUF_FLAG_DONE) {
            ESP_LOGI(TAG, "frame done");
            print_frame_head_hex(buffer[buf.index], buf.bytesused);
        }

        ESP_RETURN_ON_FALSE(ioctl(fd, VIDIOC_QBUF, &buf) == 0, ESP_FAIL, TAG,
                            "failed to requeue buffer");
    }
}

void app_main(void)
{
    int fd;
    esp_err_t ret;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ret = example_video_init();
    ESP_GOTO_ON_ERROR(ret, exit, TAG, "video init failed");

    fd = open(EXAMPLE_CAM_DEV_PATH, O_RDONLY);
    ESP_GOTO_ON_FALSE(fd >= 0, ESP_FAIL, exit, TAG, "failed to open device");

    enum_and_print_formats(fd, type);

    ret = basic_capture_loop(fd, V4L2_PIX_FMT_SBGGR10);
    ESP_GOTO_ON_ERROR(ret, close_fd, TAG, "capture loop failed");

close_fd:
    close(fd);
    ESP_ERROR_CHECK(example_video_deinit());
exit:
    return;
}
