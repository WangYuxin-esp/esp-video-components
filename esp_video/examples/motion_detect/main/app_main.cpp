/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "example_video_common.h"

#include "dl_image_motion.hpp"
#include "dl_image_process.hpp"
#include "quirc.h"

#define MAX_BUFFER_COUNT                (4)
#define MIN_BUFFER_COUNT                (2)
#define DEFAULT_BUFFER_COUNT            (2)
#define VIDEO_TASK_STACK_SIZE           (16 * 1024)
#define VIDEO_TASK_PRIORITY             (6)
#define MOV_THRESHOLD(width, height)    (width * height * 0.25)

typedef enum {
    APP_VIDEO_FMT_RAW8 = V4L2_PIX_FMT_SBGGR8,
    APP_VIDEO_FMT_RAW10 = V4L2_PIX_FMT_SBGGR10,
    APP_VIDEO_FMT_GREY = V4L2_PIX_FMT_GREY,
    APP_VIDEO_FMT_RGB565_LE = V4L2_PIX_FMT_RGB565,
    APP_VIDEO_FMT_RGB565_BE = V4L2_PIX_FMT_RGB565X,
    APP_VIDEO_FMT_RGB888 = V4L2_PIX_FMT_RGB24,
    APP_VIDEO_FMT_YUV422_YUYV = V4L2_PIX_FMT_YUYV,
    APP_VIDEO_FMT_YUV422_UYVY = V4L2_PIX_FMT_UYVY,
    APP_VIDEO_FMT_YUV420 = V4L2_PIX_FMT_YUV420,
} video_fmt_t;

typedef struct {
    uint8_t *camera_buffer[MAX_BUFFER_COUNT];
    size_t camera_buf_size;
    uint8_t camera_buf_count;
    uint32_t camera_buf_hes;
    uint32_t camera_buf_ves;
    uint8_t camera_buf_mode;
    struct v4l2_buffer v4l2_buf[DEFAULT_BUFFER_COUNT];
    TaskHandle_t video_stream_task_handle;
    uint8_t video_task_core_id;
    bool video_task_delete;
    void *video_task_user_data;
} app_video_t;

static app_video_t s_app_camera_video;
struct quirc *s_qr;
struct quirc_code s_q_code;
struct quirc_data s_q_data;
quirc_decode_error_t s_q_err;
static const char *TAG = "example";

#if 0
extern "C" void app_main(void)
{
    dl::image::jpeg_img_t dog1_jpeg = {.data = (void *)dog1_jpg_start,
                                       .data_len = (size_t)(dog1_jpg_end - dog1_jpg_start)
                                      };
    auto dog1 = dl::image::sw_decode_jpeg(dog1_jpeg, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    dl::image::jpeg_img_t dog2_jpeg = {.data = (void *)dog2_jpg_start,
                                       .data_len = (size_t)(dog2_jpg_end - dog2_jpg_start)
                                      };
    auto dog2 = dl::image::sw_decode_jpeg(dog2_jpeg, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    uint32_t n = dl::image::get_moving_point_number(dog1, dog2, 1, 5);
    if (n > MOV_THR) {
        ESP_LOGI("MOTION", "moving with %d moving pts", n);
    } else {
        ESP_LOGI("MOTION", "not moving with %d moving pts", n);
    }
    n = dl::image::get_moving_point_number(dog1, dog1, 1, 5);
    if (n > MOV_THR) {
        ESP_LOGI("MOTION", "moving with %d moving pts", n);
    } else {
        ESP_LOGI("MOTION", "not moving with %d moving pts", n);
    }
    heap_caps_free(dog1.data);
    heap_caps_free(dog2.data);
}
#endif

static int app_video_open(video_fmt_t init_fmt)
{
    struct v4l2_format default_format;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
    esp_cam_sensor_id_t chip_id;

    int fd = open(EXAMPLE_CAM_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "Open video failed");
        return -1;
    }

    controls.ctrl_class = V4L2_CTRL_CLASS_ESP_CAM_IOCTL;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = ESP_CAM_SENSOR_IOC_G_CHIP_ID;
    control[0].p_u8     = (uint8_t *)&chip_id;
    control[0].size     = sizeof(chip_id);
    if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &controls) != 0) {
        ESP_LOGE(TAG, "failed to get chip id");
    } else {
        ESP_LOGI(TAG, "chip id: 0x%" PRIx16, chip_id.pid);
    }

    memset(&default_format, 0, sizeof(struct v4l2_format));
    default_format.type = type;
    if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
        ESP_LOGE(TAG, "failed to get format");
        close(fd);
        return -1;
    }

    ESP_LOGI(TAG, "default format width=%" PRIu32 " height=%" PRIu32, default_format.fmt.pix.width, default_format.fmt.pix.height);

    s_app_camera_video.camera_buf_hes = default_format.fmt.pix.width;
    s_app_camera_video.camera_buf_ves = default_format.fmt.pix.height;

    if (default_format.fmt.pix.pixelformat != init_fmt) {
        struct v4l2_format format = {0};
        format.type = type;
        format.fmt.pix.width = default_format.fmt.pix.width;
        format.fmt.pix.height = default_format.fmt.pix.height;
        format.fmt.pix.pixelformat = init_fmt;

        if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
            ESP_LOGE(TAG, "failed to set format");
            close(fd);
            return -1;
        }
    }

    return fd;
}

esp_err_t app_video_set_bufs(int video_fd, uint32_t fb_num, const void **fb)
{
    if (fb_num > MAX_BUFFER_COUNT) {
        ESP_LOGE(TAG, "buffer num is too large");
        return ESP_FAIL;
    } else if (fb_num < MIN_BUFFER_COUNT) {
        ESP_LOGE(TAG, "At least two buffers are required");
        return ESP_FAIL;
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = fb_num;
    req.type = type;

    s_app_camera_video.camera_buf_count = fb_num;
    s_app_camera_video.camera_buf_mode = req.memory = fb ? V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP;

    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "req bufs failed");
        goto errout_req_bufs;
    }
    for (int i = 0; i < fb_num; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = req.memory;
        buf.index = i;

        if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "query buf failed");
            goto errout_req_bufs;
        }

        if (req.memory == V4L2_MEMORY_MMAP) {
            s_app_camera_video.camera_buffer[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, video_fd, buf.m.offset);
            if (s_app_camera_video.camera_buffer[i] == NULL) {
                ESP_LOGE(TAG, "mmap failed");
                goto errout_req_bufs;
            }
        } else {
            if (!fb[i]) {
                ESP_LOGE(TAG, "frame buffer is NULL");
                goto errout_req_bufs;
            }
            buf.m.userptr = (unsigned long)fb[i];
            s_app_camera_video.camera_buffer[i] = (uint8_t *)fb[i];
        }

        s_app_camera_video.camera_buf_size = buf.length;

        if (ioctl(video_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "queue frame buffer failed");
            goto errout_req_bufs;
        }
    }

    return ESP_OK;

errout_req_bufs:
    close(video_fd);
    return ESP_FAIL;
}

static inline esp_err_t video_receive_video_frame(int video_fd, uint8_t v4l2_buf_index)
{
    memset(&s_app_camera_video.v4l2_buf[v4l2_buf_index], 0, sizeof(s_app_camera_video.v4l2_buf[0]));
    s_app_camera_video.v4l2_buf[v4l2_buf_index].type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    s_app_camera_video.v4l2_buf[v4l2_buf_index].memory = s_app_camera_video.camera_buf_mode;

    int res = ioctl(video_fd, VIDIOC_DQBUF, &(s_app_camera_video.v4l2_buf[v4l2_buf_index]));
    if (res != 0) {
        ESP_LOGE(TAG, "failed to receive video frame");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static inline esp_err_t video_free_video_frame(int video_fd, uint8_t v4l2_buf_index)
{
    if (ioctl(video_fd, VIDIOC_QBUF, &(s_app_camera_video.v4l2_buf[v4l2_buf_index])) != 0) {
        ESP_LOGE(TAG, "failed to free video frame");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static inline esp_err_t video_stream_start(int video_fd)
{
    ESP_LOGI(TAG, "Video Stream Start");
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_STREAMON, &type)) {
        ESP_LOGE(TAG, "failed to start stream");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static inline esp_err_t video_stream_stop(int video_fd)
{
    ESP_LOGI(TAG, "Video Stream Stop");
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_STREAMOFF, &type)) {
        ESP_LOGE(TAG, "failed to stop stream");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t image_qr_code_process(uint8_t *src_buf, uint32_t src_width, uint32_t src_height)
{
    dl::image::img_t color = {.data = (void *)src_buf,
                              .width = (uint16_t)src_width,
                              .height = (uint16_t)src_height,
                              .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE
                             };

    uint8_t *data = quirc_begin(s_qr, NULL, NULL);
    dl::image::ImageTransformer T;

    dl::image::img_t gray = {.data = data, .width = (uint16_t)src_width, .height = (uint16_t)src_height, .pix_type = dl::image::DL_IMAGE_PIX_TYPE_GRAY};
    T.set_src_img(color).set_dst_img(gray).transform();
    quirc_end(s_qr);

    int num_codes = quirc_count(s_qr);
    for (int i = 0; i < num_codes; i++) {
        quirc_extract(s_qr, i, &s_q_code);
        s_q_err = quirc_decode(&s_q_code, &s_q_data);
        if (s_q_err == QUIRC_ERROR_DATA_ECC) {
            quirc_flip(&s_q_code);
            s_q_err = quirc_decode(&s_q_code, &s_q_data);
        }
        if (!s_q_err) {
            ESP_LOGI("qrcode", "%s", s_q_data.payload);
        }
    }
    return ESP_OK;
}

static esp_err_t image_motion_detection_process(uint8_t *src1_buf, uint8_t *src2_buf, uint32_t src_width, uint32_t src_height)
{
    dl::image::img_t color1 = {.data = (void *)src1_buf,
                               .width = (uint16_t)src_width,
                               .height = (uint16_t)src_height,
                               .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE
                              };
    dl::image::img_t color2 = {.data = (void *)src2_buf,
                               .width = (uint16_t)src_width,
                               .height = (uint16_t)src_height,
                               .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE
                              };

    uint32_t n = dl::image::get_moving_point_number(color1, color2, 1, 7);
    if (n > MOV_THRESHOLD(src_width, src_height)) {
        ESP_LOGI("MOTION", "moving with %d moving pts", n);
    } else {
        ESP_LOGI("MOTION", "not moving with %d moving pts", n);
    }
    return ESP_OK;
}

static void video_stream_task(void *arg)
{
    int video_fd = *((int *)arg);
    uint8_t v4l2_buf0_index, v4l2_buf1_index;
    const uint8_t frame1_index = 0;
    const uint8_t frame2_index = 1;

    while (1) {
        ESP_ERROR_CHECK(video_receive_video_frame(video_fd, frame1_index));
        v4l2_buf0_index = s_app_camera_video.v4l2_buf[frame1_index].index;
        ESP_ERROR_CHECK(video_receive_video_frame(video_fd, frame2_index));
        v4l2_buf1_index = s_app_camera_video.v4l2_buf[frame2_index].index;
        // image_qr_code_process(s_app_camera_video.camera_buffer[buf_index], s_app_camera_video.camera_buf_hes, s_app_camera_video.camera_buf_ves);
        image_motion_detection_process(s_app_camera_video.camera_buffer[v4l2_buf0_index], s_app_camera_video.camera_buffer[v4l2_buf1_index], s_app_camera_video.camera_buf_hes, s_app_camera_video.camera_buf_ves);
        // ESP_LOGW(TAG, "IN task");

        ESP_ERROR_CHECK(video_free_video_frame(video_fd, frame1_index));
        ESP_ERROR_CHECK(video_free_video_frame(video_fd, frame2_index));

        if (s_app_camera_video.video_task_delete) {
            s_app_camera_video.video_task_delete = false;
            ESP_ERROR_CHECK(video_stream_stop(video_fd));
            vTaskDelete(NULL);
        }
    }
    vTaskDelete(NULL);
}

esp_err_t app_video_stream_task_start(int video_fd, int core_id, void *user_data)
{
    s_app_camera_video.video_task_core_id = core_id;
    s_app_camera_video.video_task_user_data = user_data;

    video_stream_start(video_fd);

    BaseType_t result = xTaskCreatePinnedToCore(video_stream_task, "video stream task", VIDEO_TASK_STACK_SIZE, &video_fd, VIDEO_TASK_PRIORITY, &s_app_camera_video.video_stream_task_handle, core_id);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "failed to create video stream task");
        video_stream_stop(video_fd);
        return ESP_FAIL;
    }

    return ESP_OK;
}

extern "C" void app_main(void)
{
    esp_err_t ret = ESP_OK;
    int video_cam_fd0 = -1;

    ret = example_video_init();
    ESP_GOTO_ON_ERROR(ret, clean1, TAG, "Camera init failed");
    // Open the video device
    video_cam_fd0 = app_video_open(APP_VIDEO_FMT_RGB565_LE);
    if (video_cam_fd0 < 0) {
        ESP_LOGE(TAG, "video cam open failed");
        goto clean0;
    }
    ESP_ERROR_CHECK(app_video_set_bufs(video_cam_fd0, DEFAULT_BUFFER_COUNT, NULL));
    s_qr = quirc_new();
    if (!s_qr) {
        ESP_LOGE(TAG, "New qr obj failed");
        return;
    }
    if (quirc_resize(s_qr, s_app_camera_video.camera_buf_hes, s_app_camera_video.camera_buf_ves) < 0) {
        ESP_LOGE(TAG, "Failed to allocate fb memory");
        return;
    }
    ret = app_video_stream_task_start(video_cam_fd0, 0, NULL);

    ESP_GOTO_ON_ERROR(ret, clean0, TAG, "Task start failed");
    return;
clean0:
    ESP_ERROR_CHECK(example_video_deinit());
clean1:
    return;
}
