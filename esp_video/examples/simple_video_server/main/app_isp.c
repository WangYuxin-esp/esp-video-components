/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"

#include "linux/videodev2.h"
#include "esp_video_isp_ioctl.h"
#include "esp_video_device.h"

static const char *TAG = "app_isp";

#define EXP_GAIN_PARA_EN      (1)
#define BF_PARA_EN            (1)
#define LSC_PARA_EN           (1)
#define DEM_PARA_EN           (1)
#define WB_PARA_EN            (0)
#define CCM_PARA_EN           (0)
#define GAMMA_PARA_EN         (0)
#define SHARPEN_PARA_EN       (0)
#define COLOR_PARA_EN         (0)
#define BLC_PARA_EN           (1)

#if LSC_PARA_EN
// LSC Matrices: Supported resolution is (1080, 1920), calibration mode is mode1
#define LSC_F2I_F(f)      ((uint8_t)((uint32_t)((f) * 256)))
#define LSC_F2I_I(f)      ((uint8_t)((uint8_t)(f)))
int lsc_gain_size = 13 * 21;

isp_lsc_gain_t r_gain[13 * 21] = {
    {.integer = LSC_F2I_I(2.89857), .decimal = LSC_F2I_F(2.89857)}, {.integer = LSC_F2I_I(2.56229), .decimal = LSC_F2I_F(2.56229)}, {.integer = LSC_F2I_I(2.27167), .decimal = LSC_F2I_F(2.27167)}, {.integer = LSC_F2I_I(2.02107), .decimal = LSC_F2I_F(2.02107)}, {.integer = LSC_F2I_I(1.81545), .decimal = LSC_F2I_F(1.81545)}, {.integer = LSC_F2I_I(1.65325), .decimal = LSC_F2I_F(1.65325)}, {.integer = LSC_F2I_I(1.53483), .decimal = LSC_F2I_F(1.53483)}, {.integer = LSC_F2I_I(1.45393), .decimal = LSC_F2I_F(1.45393)}, {.integer = LSC_F2I_I(1.42127), .decimal = LSC_F2I_F(1.42127)}, {.integer = LSC_F2I_I(1.39306), .decimal = LSC_F2I_F(1.39306)}, {.integer = LSC_F2I_I(1.36986), .decimal = LSC_F2I_F(1.36986)}, {.integer = LSC_F2I_I(1.40141), .decimal = LSC_F2I_F(1.40141)}, {.integer = LSC_F2I_I(1.45815), .decimal = LSC_F2I_F(1.45815)}, {.integer = LSC_F2I_I(1.55767), .decimal = LSC_F2I_F(1.55767)}, {.integer = LSC_F2I_I(1.68603), .decimal = LSC_F2I_F(1.68603)}, {.integer = LSC_F2I_I(1.84826), .decimal = LSC_F2I_F(1.84826)}, {.integer = LSC_F2I_I(2.05967), .decimal = LSC_F2I_F(2.05967)}, {.integer = LSC_F2I_I(2.30768), .decimal = LSC_F2I_F(2.30768)}, {.integer = LSC_F2I_I(2.60055), .decimal = LSC_F2I_F(2.60055)}, {.integer = LSC_F2I_I(2.92594), .decimal = LSC_F2I_F(2.92594)}, {.integer = LSC_F2I_I(2.92594), .decimal = LSC_F2I_F(2.92594)},
    {.integer = LSC_F2I_I(2.68447), .decimal = LSC_F2I_F(2.68447)}, {.integer = LSC_F2I_I(2.36295), .decimal = LSC_F2I_F(2.36295)}, {.integer = LSC_F2I_I(2.07184), .decimal = LSC_F2I_F(2.07184)}, {.integer = LSC_F2I_I(1.83822), .decimal = LSC_F2I_F(1.83822)}, {.integer = LSC_F2I_I(1.64293), .decimal = LSC_F2I_F(1.64293)}, {.integer = LSC_F2I_I(1.49230), .decimal = LSC_F2I_F(1.49230)}, {.integer = LSC_F2I_I(1.37985), .decimal = LSC_F2I_F(1.37985)}, {.integer = LSC_F2I_I(1.30615), .decimal = LSC_F2I_F(1.30615)}, {.integer = LSC_F2I_I(1.27636), .decimal = LSC_F2I_F(1.27636)}, {.integer = LSC_F2I_I(1.25377), .decimal = LSC_F2I_F(1.25377)}, {.integer = LSC_F2I_I(1.22734), .decimal = LSC_F2I_F(1.22734)}, {.integer = LSC_F2I_I(1.25405), .decimal = LSC_F2I_F(1.25405)}, {.integer = LSC_F2I_I(1.30723), .decimal = LSC_F2I_F(1.30723)}, {.integer = LSC_F2I_I(1.39537), .decimal = LSC_F2I_F(1.39537)}, {.integer = LSC_F2I_I(1.51647), .decimal = LSC_F2I_F(1.51647)}, {.integer = LSC_F2I_I(1.67623), .decimal = LSC_F2I_F(1.67623)}, {.integer = LSC_F2I_I(1.87744), .decimal = LSC_F2I_F(1.87744)}, {.integer = LSC_F2I_I(2.11728), .decimal = LSC_F2I_F(2.11728)}, {.integer = LSC_F2I_I(2.40138), .decimal = LSC_F2I_F(2.40138)}, {.integer = LSC_F2I_I(2.72283), .decimal = LSC_F2I_F(2.72283)}, {.integer = LSC_F2I_I(2.72283), .decimal = LSC_F2I_F(2.72283)},
    {.integer = LSC_F2I_I(2.53156), .decimal = LSC_F2I_F(2.53156)}, {.integer = LSC_F2I_I(2.20836), .decimal = LSC_F2I_F(2.20836)}, {.integer = LSC_F2I_I(1.93018), .decimal = LSC_F2I_F(1.93018)}, {.integer = LSC_F2I_I(1.70413), .decimal = LSC_F2I_F(1.70413)}, {.integer = LSC_F2I_I(1.51781), .decimal = LSC_F2I_F(1.51781)}, {.integer = LSC_F2I_I(1.37177), .decimal = LSC_F2I_F(1.37177)}, {.integer = LSC_F2I_I(1.26717), .decimal = LSC_F2I_F(1.26717)}, {.integer = LSC_F2I_I(1.19092), .decimal = LSC_F2I_F(1.19092)}, {.integer = LSC_F2I_I(1.15163), .decimal = LSC_F2I_F(1.15163)}, {.integer = LSC_F2I_I(1.12803), .decimal = LSC_F2I_F(1.12803)}, {.integer = LSC_F2I_I(1.12394), .decimal = LSC_F2I_F(1.12394)}, {.integer = LSC_F2I_I(1.15054), .decimal = LSC_F2I_F(1.15054)}, {.integer = LSC_F2I_I(1.20306), .decimal = LSC_F2I_F(1.20306)}, {.integer = LSC_F2I_I(1.28059), .decimal = LSC_F2I_F(1.28059)}, {.integer = LSC_F2I_I(1.39058), .decimal = LSC_F2I_F(1.39058)}, {.integer = LSC_F2I_I(1.54441), .decimal = LSC_F2I_F(1.54441)}, {.integer = LSC_F2I_I(1.73584), .decimal = LSC_F2I_F(1.73584)}, {.integer = LSC_F2I_I(1.97198), .decimal = LSC_F2I_F(1.97198)}, {.integer = LSC_F2I_I(2.25182), .decimal = LSC_F2I_F(2.25182)}, {.integer = LSC_F2I_I(2.56428), .decimal = LSC_F2I_F(2.56428)}, {.integer = LSC_F2I_I(2.56428), .decimal = LSC_F2I_F(2.56428)},
    {.integer = LSC_F2I_I(2.41482), .decimal = LSC_F2I_F(2.41482)}, {.integer = LSC_F2I_I(2.09882), .decimal = LSC_F2I_F(2.09882)}, {.integer = LSC_F2I_I(1.82642), .decimal = LSC_F2I_F(1.82642)}, {.integer = LSC_F2I_I(1.60582), .decimal = LSC_F2I_F(1.60582)}, {.integer = LSC_F2I_I(1.42595), .decimal = LSC_F2I_F(1.42595)}, {.integer = LSC_F2I_I(1.28892), .decimal = LSC_F2I_F(1.28892)}, {.integer = LSC_F2I_I(1.19012), .decimal = LSC_F2I_F(1.19012)}, {.integer = LSC_F2I_I(1.11895), .decimal = LSC_F2I_F(1.11895)}, {.integer = LSC_F2I_I(1.07594), .decimal = LSC_F2I_F(1.07594)}, {.integer = LSC_F2I_I(1.05738), .decimal = LSC_F2I_F(1.05738)}, {.integer = LSC_F2I_I(1.05561), .decimal = LSC_F2I_F(1.05561)}, {.integer = LSC_F2I_I(1.07795), .decimal = LSC_F2I_F(1.07795)}, {.integer = LSC_F2I_I(1.12544), .decimal = LSC_F2I_F(1.12544)}, {.integer = LSC_F2I_I(1.20024), .decimal = LSC_F2I_F(1.20024)}, {.integer = LSC_F2I_I(1.31109), .decimal = LSC_F2I_F(1.31109)}, {.integer = LSC_F2I_I(1.45446), .decimal = LSC_F2I_F(1.45446)}, {.integer = LSC_F2I_I(1.64314), .decimal = LSC_F2I_F(1.64314)}, {.integer = LSC_F2I_I(1.87281), .decimal = LSC_F2I_F(1.87281)}, {.integer = LSC_F2I_I(2.15111), .decimal = LSC_F2I_F(2.15111)}, {.integer = LSC_F2I_I(2.45830), .decimal = LSC_F2I_F(2.45830)}, {.integer = LSC_F2I_I(2.45830), .decimal = LSC_F2I_F(2.45830)},
    {.integer = LSC_F2I_I(2.34476), .decimal = LSC_F2I_F(2.34476)}, {.integer = LSC_F2I_I(2.02833), .decimal = LSC_F2I_F(2.02833)}, {.integer = LSC_F2I_I(1.76618), .decimal = LSC_F2I_F(1.76618)}, {.integer = LSC_F2I_I(1.54691), .decimal = LSC_F2I_F(1.54691)}, {.integer = LSC_F2I_I(1.37292), .decimal = LSC_F2I_F(1.37292)}, {.integer = LSC_F2I_I(1.24120), .decimal = LSC_F2I_F(1.24120)}, {.integer = LSC_F2I_I(1.14152), .decimal = LSC_F2I_F(1.14152)}, {.integer = LSC_F2I_I(1.07508), .decimal = LSC_F2I_F(1.07508)}, {.integer = LSC_F2I_I(1.03194), .decimal = LSC_F2I_F(1.03194)}, {.integer = LSC_F2I_I(1.01204), .decimal = LSC_F2I_F(1.01204)}, {.integer = LSC_F2I_I(1.01373), .decimal = LSC_F2I_F(1.01373)}, {.integer = LSC_F2I_I(1.03448), .decimal = LSC_F2I_F(1.03448)}, {.integer = LSC_F2I_I(1.07825), .decimal = LSC_F2I_F(1.07825)}, {.integer = LSC_F2I_I(1.15243), .decimal = LSC_F2I_F(1.15243)}, {.integer = LSC_F2I_I(1.25938), .decimal = LSC_F2I_F(1.25938)}, {.integer = LSC_F2I_I(1.40311), .decimal = LSC_F2I_F(1.40311)}, {.integer = LSC_F2I_I(1.58683), .decimal = LSC_F2I_F(1.58683)}, {.integer = LSC_F2I_I(1.81553), .decimal = LSC_F2I_F(1.81553)}, {.integer = LSC_F2I_I(2.07992), .decimal = LSC_F2I_F(2.07992)}, {.integer = LSC_F2I_I(2.39605), .decimal = LSC_F2I_F(2.39605)}, {.integer = LSC_F2I_I(2.39605), .decimal = LSC_F2I_F(2.39605)},
    {.integer = LSC_F2I_I(2.31028), .decimal = LSC_F2I_F(2.31028)}, {.integer = LSC_F2I_I(2.00200), .decimal = LSC_F2I_F(2.00200)}, {.integer = LSC_F2I_I(1.73856), .decimal = LSC_F2I_F(1.73856)}, {.integer = LSC_F2I_I(1.52096), .decimal = LSC_F2I_F(1.52096)}, {.integer = LSC_F2I_I(1.35047), .decimal = LSC_F2I_F(1.35047)}, {.integer = LSC_F2I_I(1.21876), .decimal = LSC_F2I_F(1.21876)}, {.integer = LSC_F2I_I(1.12237), .decimal = LSC_F2I_F(1.12237)}, {.integer = LSC_F2I_I(1.05625), .decimal = LSC_F2I_F(1.05625)}, {.integer = LSC_F2I_I(1.01466), .decimal = LSC_F2I_F(1.01466)}, {.integer = LSC_F2I_I(0.99310), .decimal = LSC_F2I_F(0.99310)}, {.integer = LSC_F2I_I(0.99567), .decimal = LSC_F2I_F(0.99567)}, {.integer = LSC_F2I_I(1.01562), .decimal = LSC_F2I_F(1.01562)}, {.integer = LSC_F2I_I(1.05962), .decimal = LSC_F2I_F(1.05962)}, {.integer = LSC_F2I_I(1.13355), .decimal = LSC_F2I_F(1.13355)}, {.integer = LSC_F2I_I(1.23847), .decimal = LSC_F2I_F(1.23847)}, {.integer = LSC_F2I_I(1.37952), .decimal = LSC_F2I_F(1.37952)}, {.integer = LSC_F2I_I(1.56185), .decimal = LSC_F2I_F(1.56185)}, {.integer = LSC_F2I_I(1.78637), .decimal = LSC_F2I_F(1.78637)}, {.integer = LSC_F2I_I(2.05558), .decimal = LSC_F2I_F(2.05558)}, {.integer = LSC_F2I_I(2.36483), .decimal = LSC_F2I_F(2.36483)}, {.integer = LSC_F2I_I(2.36483), .decimal = LSC_F2I_F(2.36483)},
    {.integer = LSC_F2I_I(2.32120), .decimal = LSC_F2I_F(2.32120)}, {.integer = LSC_F2I_I(2.01226), .decimal = LSC_F2I_F(2.01226)}, {.integer = LSC_F2I_I(1.74516), .decimal = LSC_F2I_F(1.74516)}, {.integer = LSC_F2I_I(1.53117), .decimal = LSC_F2I_F(1.53117)}, {.integer = LSC_F2I_I(1.36238), .decimal = LSC_F2I_F(1.36238)}, {.integer = LSC_F2I_I(1.22783), .decimal = LSC_F2I_F(1.22783)}, {.integer = LSC_F2I_I(1.12872), .decimal = LSC_F2I_F(1.12872)}, {.integer = LSC_F2I_I(1.06020), .decimal = LSC_F2I_F(1.06020)}, {.integer = LSC_F2I_I(1.01740), .decimal = LSC_F2I_F(1.01740)}, {.integer = LSC_F2I_I(0.99770), .decimal = LSC_F2I_F(0.99770)}, {.integer = LSC_F2I_I(1.00000), .decimal = LSC_F2I_F(1.00000)}, {.integer = LSC_F2I_I(1.02194), .decimal = LSC_F2I_F(1.02194)}, {.integer = LSC_F2I_I(1.06629), .decimal = LSC_F2I_F(1.06629)}, {.integer = LSC_F2I_I(1.13994), .decimal = LSC_F2I_F(1.13994)}, {.integer = LSC_F2I_I(1.24614), .decimal = LSC_F2I_F(1.24614)}, {.integer = LSC_F2I_I(1.38811), .decimal = LSC_F2I_F(1.38811)}, {.integer = LSC_F2I_I(1.57195), .decimal = LSC_F2I_F(1.57195)}, {.integer = LSC_F2I_I(1.79806), .decimal = LSC_F2I_F(1.79806)}, {.integer = LSC_F2I_I(2.06272), .decimal = LSC_F2I_F(2.06272)}, {.integer = LSC_F2I_I(2.37217), .decimal = LSC_F2I_F(2.37217)}, {.integer = LSC_F2I_I(2.37217), .decimal = LSC_F2I_F(2.37217)},
    {.integer = LSC_F2I_I(2.37146), .decimal = LSC_F2I_F(2.37146)}, {.integer = LSC_F2I_I(2.05889), .decimal = LSC_F2I_F(2.05889)}, {.integer = LSC_F2I_I(1.79376), .decimal = LSC_F2I_F(1.79376)}, {.integer = LSC_F2I_I(1.57591), .decimal = LSC_F2I_F(1.57591)}, {.integer = LSC_F2I_I(1.40004), .decimal = LSC_F2I_F(1.40004)}, {.integer = LSC_F2I_I(1.26387), .decimal = LSC_F2I_F(1.26387)}, {.integer = LSC_F2I_I(1.16275), .decimal = LSC_F2I_F(1.16275)}, {.integer = LSC_F2I_I(1.09109), .decimal = LSC_F2I_F(1.09109)}, {.integer = LSC_F2I_I(1.04902), .decimal = LSC_F2I_F(1.04902)}, {.integer = LSC_F2I_I(1.02680), .decimal = LSC_F2I_F(1.02680)}, {.integer = LSC_F2I_I(1.02783), .decimal = LSC_F2I_F(1.02783)}, {.integer = LSC_F2I_I(1.05176), .decimal = LSC_F2I_F(1.05176)}, {.integer = LSC_F2I_I(1.10066), .decimal = LSC_F2I_F(1.10066)}, {.integer = LSC_F2I_I(1.17301), .decimal = LSC_F2I_F(1.17301)}, {.integer = LSC_F2I_I(1.28692), .decimal = LSC_F2I_F(1.28692)}, {.integer = LSC_F2I_I(1.43367), .decimal = LSC_F2I_F(1.43367)}, {.integer = LSC_F2I_I(1.61738), .decimal = LSC_F2I_F(1.61738)}, {.integer = LSC_F2I_I(1.84341), .decimal = LSC_F2I_F(1.84341)}, {.integer = LSC_F2I_I(2.11932), .decimal = LSC_F2I_F(2.11932)}, {.integer = LSC_F2I_I(2.42572), .decimal = LSC_F2I_F(2.42572)}, {.integer = LSC_F2I_I(2.42572), .decimal = LSC_F2I_F(2.42572)},
    {.integer = LSC_F2I_I(2.45699), .decimal = LSC_F2I_F(2.45699)}, {.integer = LSC_F2I_I(2.14518), .decimal = LSC_F2I_F(2.14518)}, {.integer = LSC_F2I_I(1.87806), .decimal = LSC_F2I_F(1.87806)}, {.integer = LSC_F2I_I(1.65341), .decimal = LSC_F2I_F(1.65341)}, {.integer = LSC_F2I_I(1.47082), .decimal = LSC_F2I_F(1.47082)}, {.integer = LSC_F2I_I(1.33143), .decimal = LSC_F2I_F(1.33143)}, {.integer = LSC_F2I_I(1.22653), .decimal = LSC_F2I_F(1.22653)}, {.integer = LSC_F2I_I(1.15388), .decimal = LSC_F2I_F(1.15388)}, {.integer = LSC_F2I_I(1.10664), .decimal = LSC_F2I_F(1.10664)}, {.integer = LSC_F2I_I(1.08390), .decimal = LSC_F2I_F(1.08390)}, {.integer = LSC_F2I_I(1.08593), .decimal = LSC_F2I_F(1.08593)}, {.integer = LSC_F2I_I(1.10949), .decimal = LSC_F2I_F(1.10949)}, {.integer = LSC_F2I_I(1.16240), .decimal = LSC_F2I_F(1.16240)}, {.integer = LSC_F2I_I(1.24510), .decimal = LSC_F2I_F(1.24510)}, {.integer = LSC_F2I_I(1.35706), .decimal = LSC_F2I_F(1.35706)}, {.integer = LSC_F2I_I(1.50794), .decimal = LSC_F2I_F(1.50794)}, {.integer = LSC_F2I_I(1.70071), .decimal = LSC_F2I_F(1.70071)}, {.integer = LSC_F2I_I(1.93098), .decimal = LSC_F2I_F(1.93098)}, {.integer = LSC_F2I_I(2.20657), .decimal = LSC_F2I_F(2.20657)}, {.integer = LSC_F2I_I(2.51756), .decimal = LSC_F2I_F(2.51756)}, {.integer = LSC_F2I_I(2.51756), .decimal = LSC_F2I_F(2.51756)},
    {.integer = LSC_F2I_I(2.59893), .decimal = LSC_F2I_F(2.59893)}, {.integer = LSC_F2I_I(2.27655), .decimal = LSC_F2I_F(2.27655)}, {.integer = LSC_F2I_I(1.99842), .decimal = LSC_F2I_F(1.99842)}, {.integer = LSC_F2I_I(1.77148), .decimal = LSC_F2I_F(1.77148)}, {.integer = LSC_F2I_I(1.58557), .decimal = LSC_F2I_F(1.58557)}, {.integer = LSC_F2I_I(1.43433), .decimal = LSC_F2I_F(1.43433)}, {.integer = LSC_F2I_I(1.32676), .decimal = LSC_F2I_F(1.32676)}, {.integer = LSC_F2I_I(1.24886), .decimal = LSC_F2I_F(1.24886)}, {.integer = LSC_F2I_I(1.19907), .decimal = LSC_F2I_F(1.19907)}, {.integer = LSC_F2I_I(1.17385), .decimal = LSC_F2I_F(1.17385)}, {.integer = LSC_F2I_I(1.17469), .decimal = LSC_F2I_F(1.17469)}, {.integer = LSC_F2I_I(1.20504), .decimal = LSC_F2I_F(1.20504)}, {.integer = LSC_F2I_I(1.26034), .decimal = LSC_F2I_F(1.26034)}, {.integer = LSC_F2I_I(1.34629), .decimal = LSC_F2I_F(1.34629)}, {.integer = LSC_F2I_I(1.46372), .decimal = LSC_F2I_F(1.46372)}, {.integer = LSC_F2I_I(1.62337), .decimal = LSC_F2I_F(1.62337)}, {.integer = LSC_F2I_I(1.81791), .decimal = LSC_F2I_F(1.81791)}, {.integer = LSC_F2I_I(2.06398), .decimal = LSC_F2I_F(2.06398)}, {.integer = LSC_F2I_I(2.33492), .decimal = LSC_F2I_F(2.33492)}, {.integer = LSC_F2I_I(2.65077), .decimal = LSC_F2I_F(2.65077)}, {.integer = LSC_F2I_I(2.65077), .decimal = LSC_F2I_F(2.65077)},
    {.integer = LSC_F2I_I(2.76981), .decimal = LSC_F2I_F(2.76981)}, {.integer = LSC_F2I_I(2.45165), .decimal = LSC_F2I_F(2.45165)}, {.integer = LSC_F2I_I(2.16956), .decimal = LSC_F2I_F(2.16956)}, {.integer = LSC_F2I_I(1.93453), .decimal = LSC_F2I_F(1.93453)}, {.integer = LSC_F2I_I(1.73361), .decimal = LSC_F2I_F(1.73361)}, {.integer = LSC_F2I_I(1.57937), .decimal = LSC_F2I_F(1.57937)}, {.integer = LSC_F2I_I(1.46164), .decimal = LSC_F2I_F(1.46164)}, {.integer = LSC_F2I_I(1.38068), .decimal = LSC_F2I_F(1.38068)}, {.integer = LSC_F2I_I(1.32756), .decimal = LSC_F2I_F(1.32756)}, {.integer = LSC_F2I_I(1.30048), .decimal = LSC_F2I_F(1.30048)}, {.integer = LSC_F2I_I(1.30487), .decimal = LSC_F2I_F(1.30487)}, {.integer = LSC_F2I_I(1.33823), .decimal = LSC_F2I_F(1.33823)}, {.integer = LSC_F2I_I(1.39537), .decimal = LSC_F2I_F(1.39537)}, {.integer = LSC_F2I_I(1.48810), .decimal = LSC_F2I_F(1.48810)}, {.integer = LSC_F2I_I(1.61373), .decimal = LSC_F2I_F(1.61373)}, {.integer = LSC_F2I_I(1.77890), .decimal = LSC_F2I_F(1.77890)}, {.integer = LSC_F2I_I(1.98147), .decimal = LSC_F2I_F(1.98147)}, {.integer = LSC_F2I_I(2.22817), .decimal = LSC_F2I_F(2.22817)}, {.integer = LSC_F2I_I(2.51135), .decimal = LSC_F2I_F(2.51135)}, {.integer = LSC_F2I_I(2.83267), .decimal = LSC_F2I_F(2.83267)}, {.integer = LSC_F2I_I(2.83267), .decimal = LSC_F2I_F(2.83267)},
    {.integer = LSC_F2I_I(2.91355), .decimal = LSC_F2I_F(2.91355)}, {.integer = LSC_F2I_I(2.57764), .decimal = LSC_F2I_F(2.57764)}, {.integer = LSC_F2I_I(2.30505), .decimal = LSC_F2I_F(2.30505)}, {.integer = LSC_F2I_I(2.04804), .decimal = LSC_F2I_F(2.04804)}, {.integer = LSC_F2I_I(1.85066), .decimal = LSC_F2I_F(1.85066)}, {.integer = LSC_F2I_I(1.69200), .decimal = LSC_F2I_F(1.69200)}, {.integer = LSC_F2I_I(1.56736), .decimal = LSC_F2I_F(1.56736)}, {.integer = LSC_F2I_I(1.48794), .decimal = LSC_F2I_F(1.48794)}, {.integer = LSC_F2I_I(1.43008), .decimal = LSC_F2I_F(1.43008)}, {.integer = LSC_F2I_I(1.40644), .decimal = LSC_F2I_F(1.40644)}, {.integer = LSC_F2I_I(1.40682), .decimal = LSC_F2I_F(1.40682)}, {.integer = LSC_F2I_I(1.44062), .decimal = LSC_F2I_F(1.44062)}, {.integer = LSC_F2I_I(1.49977), .decimal = LSC_F2I_F(1.49977)}, {.integer = LSC_F2I_I(1.59638), .decimal = LSC_F2I_F(1.59638)}, {.integer = LSC_F2I_I(1.72834), .decimal = LSC_F2I_F(1.72834)}, {.integer = LSC_F2I_I(1.90437), .decimal = LSC_F2I_F(1.90437)}, {.integer = LSC_F2I_I(2.10932), .decimal = LSC_F2I_F(2.10932)}, {.integer = LSC_F2I_I(2.35216), .decimal = LSC_F2I_F(2.35216)}, {.integer = LSC_F2I_I(2.64920), .decimal = LSC_F2I_F(2.64920)}, {.integer = LSC_F2I_I(2.96657), .decimal = LSC_F2I_F(2.96657)}, {.integer = LSC_F2I_I(2.96657), .decimal = LSC_F2I_F(2.96657)},
    {.integer = LSC_F2I_I(2.91355), .decimal = LSC_F2I_F(2.91355)}, {.integer = LSC_F2I_I(2.57764), .decimal = LSC_F2I_F(2.57764)}, {.integer = LSC_F2I_I(2.30505), .decimal = LSC_F2I_F(2.30505)}, {.integer = LSC_F2I_I(2.04804), .decimal = LSC_F2I_F(2.04804)}, {.integer = LSC_F2I_I(1.85066), .decimal = LSC_F2I_F(1.85066)}, {.integer = LSC_F2I_I(1.69200), .decimal = LSC_F2I_F(1.69200)}, {.integer = LSC_F2I_I(1.56736), .decimal = LSC_F2I_F(1.56736)}, {.integer = LSC_F2I_I(1.48794), .decimal = LSC_F2I_F(1.48794)}, {.integer = LSC_F2I_I(1.43008), .decimal = LSC_F2I_F(1.43008)}, {.integer = LSC_F2I_I(1.40644), .decimal = LSC_F2I_F(1.40644)}, {.integer = LSC_F2I_I(1.40682), .decimal = LSC_F2I_F(1.40682)}, {.integer = LSC_F2I_I(1.44062), .decimal = LSC_F2I_F(1.44062)}, {.integer = LSC_F2I_I(1.49977), .decimal = LSC_F2I_F(1.49977)}, {.integer = LSC_F2I_I(1.59638), .decimal = LSC_F2I_F(1.59638)}, {.integer = LSC_F2I_I(1.72834), .decimal = LSC_F2I_F(1.72834)}, {.integer = LSC_F2I_I(1.90437), .decimal = LSC_F2I_F(1.90437)}, {.integer = LSC_F2I_I(2.10932), .decimal = LSC_F2I_F(2.10932)}, {.integer = LSC_F2I_I(2.35216), .decimal = LSC_F2I_F(2.35216)}, {.integer = LSC_F2I_I(2.64920), .decimal = LSC_F2I_F(2.64920)}, {.integer = LSC_F2I_I(2.96657), .decimal = LSC_F2I_F(2.96657)}, {.integer = LSC_F2I_I(2.96657), .decimal = LSC_F2I_F(2.96657)}
};

isp_lsc_gain_t gr_gain[13 * 21] = {
    {.integer = LSC_F2I_I(2.20976), .decimal = LSC_F2I_F(2.20976)}, {.integer = LSC_F2I_I(2.01527), .decimal = LSC_F2I_F(2.01527)}, {.integer = LSC_F2I_I(1.83965), .decimal = LSC_F2I_F(1.83965)}, {.integer = LSC_F2I_I(1.69322), .decimal = LSC_F2I_F(1.69322)}, {.integer = LSC_F2I_I(1.56522), .decimal = LSC_F2I_F(1.56522)}, {.integer = LSC_F2I_I(1.46827), .decimal = LSC_F2I_F(1.46827)}, {.integer = LSC_F2I_I(1.39091), .decimal = LSC_F2I_F(1.39091)}, {.integer = LSC_F2I_I(1.33438), .decimal = LSC_F2I_F(1.33438)}, {.integer = LSC_F2I_I(1.32171), .decimal = LSC_F2I_F(1.32171)}, {.integer = LSC_F2I_I(1.30181), .decimal = LSC_F2I_F(1.30181)}, {.integer = LSC_F2I_I(1.27211), .decimal = LSC_F2I_F(1.27211)}, {.integer = LSC_F2I_I(1.28856), .decimal = LSC_F2I_F(1.28856)}, {.integer = LSC_F2I_I(1.32577), .decimal = LSC_F2I_F(1.32577)}, {.integer = LSC_F2I_I(1.38377), .decimal = LSC_F2I_F(1.38377)}, {.integer = LSC_F2I_I(1.45927), .decimal = LSC_F2I_F(1.45927)}, {.integer = LSC_F2I_I(1.54950), .decimal = LSC_F2I_F(1.54950)}, {.integer = LSC_F2I_I(1.66905), .decimal = LSC_F2I_F(1.66905)}, {.integer = LSC_F2I_I(1.80757), .decimal = LSC_F2I_F(1.80757)}, {.integer = LSC_F2I_I(1.96825), .decimal = LSC_F2I_F(1.96825)}, {.integer = LSC_F2I_I(2.14445), .decimal = LSC_F2I_F(2.14445)}, {.integer = LSC_F2I_I(2.14445), .decimal = LSC_F2I_F(2.14445)},
    {.integer = LSC_F2I_I(2.08809), .decimal = LSC_F2I_F(2.08809)}, {.integer = LSC_F2I_I(1.89191), .decimal = LSC_F2I_F(1.89191)}, {.integer = LSC_F2I_I(1.72151), .decimal = LSC_F2I_F(1.72151)}, {.integer = LSC_F2I_I(1.57632), .decimal = LSC_F2I_F(1.57632)}, {.integer = LSC_F2I_I(1.45556), .decimal = LSC_F2I_F(1.45556)}, {.integer = LSC_F2I_I(1.35929), .decimal = LSC_F2I_F(1.35929)}, {.integer = LSC_F2I_I(1.28434), .decimal = LSC_F2I_F(1.28434)}, {.integer = LSC_F2I_I(1.23545), .decimal = LSC_F2I_F(1.23545)}, {.integer = LSC_F2I_I(1.21784), .decimal = LSC_F2I_F(1.21784)}, {.integer = LSC_F2I_I(1.20050), .decimal = LSC_F2I_F(1.20050)}, {.integer = LSC_F2I_I(1.17336), .decimal = LSC_F2I_F(1.17336)}, {.integer = LSC_F2I_I(1.18890), .decimal = LSC_F2I_F(1.18890)}, {.integer = LSC_F2I_I(1.22331), .decimal = LSC_F2I_F(1.22331)}, {.integer = LSC_F2I_I(1.27481), .decimal = LSC_F2I_F(1.27481)}, {.integer = LSC_F2I_I(1.34790), .decimal = LSC_F2I_F(1.34790)}, {.integer = LSC_F2I_I(1.44209), .decimal = LSC_F2I_F(1.44209)}, {.integer = LSC_F2I_I(1.55838), .decimal = LSC_F2I_F(1.55838)}, {.integer = LSC_F2I_I(1.69873), .decimal = LSC_F2I_F(1.69873)}, {.integer = LSC_F2I_I(1.85782), .decimal = LSC_F2I_F(1.85782)}, {.integer = LSC_F2I_I(2.03662), .decimal = LSC_F2I_F(2.03662)}, {.integer = LSC_F2I_I(2.03662), .decimal = LSC_F2I_F(2.03662)},
    {.integer = LSC_F2I_I(1.99150), .decimal = LSC_F2I_F(1.99150)}, {.integer = LSC_F2I_I(1.79761), .decimal = LSC_F2I_F(1.79761)}, {.integer = LSC_F2I_I(1.62768), .decimal = LSC_F2I_F(1.62768)}, {.integer = LSC_F2I_I(1.48591), .decimal = LSC_F2I_F(1.48591)}, {.integer = LSC_F2I_I(1.36951), .decimal = LSC_F2I_F(1.36951)}, {.integer = LSC_F2I_I(1.27397), .decimal = LSC_F2I_F(1.27397)}, {.integer = LSC_F2I_I(1.20228), .decimal = LSC_F2I_F(1.20228)}, {.integer = LSC_F2I_I(1.14968), .decimal = LSC_F2I_F(1.14968)}, {.integer = LSC_F2I_I(1.12367), .decimal = LSC_F2I_F(1.12367)}, {.integer = LSC_F2I_I(1.10547), .decimal = LSC_F2I_F(1.10547)}, {.integer = LSC_F2I_I(1.10021), .decimal = LSC_F2I_F(1.10021)}, {.integer = LSC_F2I_I(1.11500), .decimal = LSC_F2I_F(1.11500)}, {.integer = LSC_F2I_I(1.14539), .decimal = LSC_F2I_F(1.14539)}, {.integer = LSC_F2I_I(1.19523), .decimal = LSC_F2I_F(1.19523)}, {.integer = LSC_F2I_I(1.26561), .decimal = LSC_F2I_F(1.26561)}, {.integer = LSC_F2I_I(1.35790), .decimal = LSC_F2I_F(1.35790)}, {.integer = LSC_F2I_I(1.47561), .decimal = LSC_F2I_F(1.47561)}, {.integer = LSC_F2I_I(1.61159), .decimal = LSC_F2I_F(1.61159)}, {.integer = LSC_F2I_I(1.77010), .decimal = LSC_F2I_F(1.77010)}, {.integer = LSC_F2I_I(1.94898), .decimal = LSC_F2I_F(1.94898)}, {.integer = LSC_F2I_I(1.94898), .decimal = LSC_F2I_F(1.94898)},
    {.integer = LSC_F2I_I(1.92056), .decimal = LSC_F2I_F(1.92056)}, {.integer = LSC_F2I_I(1.72668), .decimal = LSC_F2I_F(1.72668)}, {.integer = LSC_F2I_I(1.56032), .decimal = LSC_F2I_F(1.56032)}, {.integer = LSC_F2I_I(1.42235), .decimal = LSC_F2I_F(1.42235)}, {.integer = LSC_F2I_I(1.30767), .decimal = LSC_F2I_F(1.30767)}, {.integer = LSC_F2I_I(1.21450), .decimal = LSC_F2I_F(1.21450)}, {.integer = LSC_F2I_I(1.14505), .decimal = LSC_F2I_F(1.14505)}, {.integer = LSC_F2I_I(1.09471), .decimal = LSC_F2I_F(1.09471)}, {.integer = LSC_F2I_I(1.06509), .decimal = LSC_F2I_F(1.06509)}, {.integer = LSC_F2I_I(1.04975), .decimal = LSC_F2I_F(1.04975)}, {.integer = LSC_F2I_I(1.04860), .decimal = LSC_F2I_F(1.04860)}, {.integer = LSC_F2I_I(1.06085), .decimal = LSC_F2I_F(1.06085)}, {.integer = LSC_F2I_I(1.08977), .decimal = LSC_F2I_F(1.08977)}, {.integer = LSC_F2I_I(1.14023), .decimal = LSC_F2I_F(1.14023)}, {.integer = LSC_F2I_I(1.20587), .decimal = LSC_F2I_F(1.20587)}, {.integer = LSC_F2I_I(1.29808), .decimal = LSC_F2I_F(1.29808)}, {.integer = LSC_F2I_I(1.41194), .decimal = LSC_F2I_F(1.41194)}, {.integer = LSC_F2I_I(1.54778), .decimal = LSC_F2I_F(1.54778)}, {.integer = LSC_F2I_I(1.70618), .decimal = LSC_F2I_F(1.70618)}, {.integer = LSC_F2I_I(1.88484), .decimal = LSC_F2I_F(1.88484)}, {.integer = LSC_F2I_I(1.88484), .decimal = LSC_F2I_F(1.88484)},
    {.integer = LSC_F2I_I(1.87622), .decimal = LSC_F2I_F(1.87622)}, {.integer = LSC_F2I_I(1.68377), .decimal = LSC_F2I_F(1.68377)}, {.integer = LSC_F2I_I(1.51798), .decimal = LSC_F2I_F(1.51798)}, {.integer = LSC_F2I_I(1.38068), .decimal = LSC_F2I_F(1.38068)}, {.integer = LSC_F2I_I(1.26528), .decimal = LSC_F2I_F(1.26528)}, {.integer = LSC_F2I_I(1.17551), .decimal = LSC_F2I_F(1.17551)}, {.integer = LSC_F2I_I(1.10779), .decimal = LSC_F2I_F(1.10779)}, {.integer = LSC_F2I_I(1.05970), .decimal = LSC_F2I_F(1.05970)}, {.integer = LSC_F2I_I(1.03083), .decimal = LSC_F2I_F(1.03083)}, {.integer = LSC_F2I_I(1.01584), .decimal = LSC_F2I_F(1.01584)}, {.integer = LSC_F2I_I(1.01616), .decimal = LSC_F2I_F(1.01616)}, {.integer = LSC_F2I_I(1.02638), .decimal = LSC_F2I_F(1.02638)}, {.integer = LSC_F2I_I(1.05521), .decimal = LSC_F2I_F(1.05521)}, {.integer = LSC_F2I_I(1.10166), .decimal = LSC_F2I_F(1.10166)}, {.integer = LSC_F2I_I(1.16906), .decimal = LSC_F2I_F(1.16906)}, {.integer = LSC_F2I_I(1.26027), .decimal = LSC_F2I_F(1.26027)}, {.integer = LSC_F2I_I(1.37283), .decimal = LSC_F2I_F(1.37283)}, {.integer = LSC_F2I_I(1.50884), .decimal = LSC_F2I_F(1.50884)}, {.integer = LSC_F2I_I(1.66339), .decimal = LSC_F2I_F(1.66339)}, {.integer = LSC_F2I_I(1.84820), .decimal = LSC_F2I_F(1.84820)}, {.integer = LSC_F2I_I(1.84820), .decimal = LSC_F2I_F(1.84820)},
    {.integer = LSC_F2I_I(1.85478), .decimal = LSC_F2I_F(1.85478)}, {.integer = LSC_F2I_I(1.66251), .decimal = LSC_F2I_F(1.66251)}, {.integer = LSC_F2I_I(1.49980), .decimal = LSC_F2I_F(1.49980)}, {.integer = LSC_F2I_I(1.35902), .decimal = LSC_F2I_F(1.35902)}, {.integer = LSC_F2I_I(1.24765), .decimal = LSC_F2I_F(1.24765)}, {.integer = LSC_F2I_I(1.15795), .decimal = LSC_F2I_F(1.15795)}, {.integer = LSC_F2I_I(1.09195), .decimal = LSC_F2I_F(1.09195)}, {.integer = LSC_F2I_I(1.04371), .decimal = LSC_F2I_F(1.04371)}, {.integer = LSC_F2I_I(1.01470), .decimal = LSC_F2I_F(1.01470)}, {.integer = LSC_F2I_I(1.00107), .decimal = LSC_F2I_F(1.00107)}, {.integer = LSC_F2I_I(0.99941), .decimal = LSC_F2I_F(0.99941)}, {.integer = LSC_F2I_I(1.01049), .decimal = LSC_F2I_F(1.01049)}, {.integer = LSC_F2I_I(1.03892), .decimal = LSC_F2I_F(1.03892)}, {.integer = LSC_F2I_I(1.08550), .decimal = LSC_F2I_F(1.08550)}, {.integer = LSC_F2I_I(1.15319), .decimal = LSC_F2I_F(1.15319)}, {.integer = LSC_F2I_I(1.24441), .decimal = LSC_F2I_F(1.24441)}, {.integer = LSC_F2I_I(1.35558), .decimal = LSC_F2I_F(1.35558)}, {.integer = LSC_F2I_I(1.49078), .decimal = LSC_F2I_F(1.49078)}, {.integer = LSC_F2I_I(1.64707), .decimal = LSC_F2I_F(1.64707)}, {.integer = LSC_F2I_I(1.82695), .decimal = LSC_F2I_F(1.82695)}, {.integer = LSC_F2I_I(1.82695), .decimal = LSC_F2I_F(1.82695)},
    {.integer = LSC_F2I_I(1.85522), .decimal = LSC_F2I_F(1.85522)}, {.integer = LSC_F2I_I(1.66537), .decimal = LSC_F2I_F(1.66537)}, {.integer = LSC_F2I_I(1.50211), .decimal = LSC_F2I_F(1.50211)}, {.integer = LSC_F2I_I(1.36419), .decimal = LSC_F2I_F(1.36419)}, {.integer = LSC_F2I_I(1.25068), .decimal = LSC_F2I_F(1.25068)}, {.integer = LSC_F2I_I(1.16066), .decimal = LSC_F2I_F(1.16066)}, {.integer = LSC_F2I_I(1.09488), .decimal = LSC_F2I_F(1.09488)}, {.integer = LSC_F2I_I(1.04586), .decimal = LSC_F2I_F(1.04586)}, {.integer = LSC_F2I_I(1.01490), .decimal = LSC_F2I_F(1.01490)}, {.integer = LSC_F2I_I(1.00167), .decimal = LSC_F2I_F(1.00167)}, {.integer = LSC_F2I_I(1.00000), .decimal = LSC_F2I_F(1.00000)}, {.integer = LSC_F2I_I(1.01315), .decimal = LSC_F2I_F(1.01315)}, {.integer = LSC_F2I_I(1.04323), .decimal = LSC_F2I_F(1.04323)}, {.integer = LSC_F2I_I(1.08873), .decimal = LSC_F2I_F(1.08873)}, {.integer = LSC_F2I_I(1.15769), .decimal = LSC_F2I_F(1.15769)}, {.integer = LSC_F2I_I(1.24781), .decimal = LSC_F2I_F(1.24781)}, {.integer = LSC_F2I_I(1.36116), .decimal = LSC_F2I_F(1.36116)}, {.integer = LSC_F2I_I(1.49635), .decimal = LSC_F2I_F(1.49635)}, {.integer = LSC_F2I_I(1.65490), .decimal = LSC_F2I_F(1.65490)}, {.integer = LSC_F2I_I(1.83155), .decimal = LSC_F2I_F(1.83155)}, {.integer = LSC_F2I_I(1.83155), .decimal = LSC_F2I_F(1.83155)},
    {.integer = LSC_F2I_I(1.87870), .decimal = LSC_F2I_F(1.87870)}, {.integer = LSC_F2I_I(1.69146), .decimal = LSC_F2I_F(1.69146)}, {.integer = LSC_F2I_I(1.52725), .decimal = LSC_F2I_F(1.52725)}, {.integer = LSC_F2I_I(1.39050), .decimal = LSC_F2I_F(1.39050)}, {.integer = LSC_F2I_I(1.27501), .decimal = LSC_F2I_F(1.27501)}, {.integer = LSC_F2I_I(1.18565), .decimal = LSC_F2I_F(1.18565)}, {.integer = LSC_F2I_I(1.11642), .decimal = LSC_F2I_F(1.11642)}, {.integer = LSC_F2I_I(1.06748), .decimal = LSC_F2I_F(1.06748)}, {.integer = LSC_F2I_I(1.03534), .decimal = LSC_F2I_F(1.03534)}, {.integer = LSC_F2I_I(1.01962), .decimal = LSC_F2I_F(1.01962)}, {.integer = LSC_F2I_I(1.01976), .decimal = LSC_F2I_F(1.01976)}, {.integer = LSC_F2I_I(1.03239), .decimal = LSC_F2I_F(1.03239)}, {.integer = LSC_F2I_I(1.06449), .decimal = LSC_F2I_F(1.06449)}, {.integer = LSC_F2I_I(1.11444), .decimal = LSC_F2I_F(1.11444)}, {.integer = LSC_F2I_I(1.18526), .decimal = LSC_F2I_F(1.18526)}, {.integer = LSC_F2I_I(1.27655), .decimal = LSC_F2I_F(1.27655)}, {.integer = LSC_F2I_I(1.38878), .decimal = LSC_F2I_F(1.38878)}, {.integer = LSC_F2I_I(1.52594), .decimal = LSC_F2I_F(1.52594)}, {.integer = LSC_F2I_I(1.68467), .decimal = LSC_F2I_F(1.68467)}, {.integer = LSC_F2I_I(1.86204), .decimal = LSC_F2I_F(1.86204)}, {.integer = LSC_F2I_I(1.86204), .decimal = LSC_F2I_F(1.86204)},
    {.integer = LSC_F2I_I(1.93081), .decimal = LSC_F2I_F(1.93081)}, {.integer = LSC_F2I_I(1.74141), .decimal = LSC_F2I_F(1.74141)}, {.integer = LSC_F2I_I(1.57583), .decimal = LSC_F2I_F(1.57583)}, {.integer = LSC_F2I_I(1.43682), .decimal = LSC_F2I_F(1.43682)}, {.integer = LSC_F2I_I(1.32126), .decimal = LSC_F2I_F(1.32126)}, {.integer = LSC_F2I_I(1.22948), .decimal = LSC_F2I_F(1.22948)}, {.integer = LSC_F2I_I(1.15854), .decimal = LSC_F2I_F(1.15854)}, {.integer = LSC_F2I_I(1.10720), .decimal = LSC_F2I_F(1.10720)}, {.integer = LSC_F2I_I(1.07428), .decimal = LSC_F2I_F(1.07428)}, {.integer = LSC_F2I_I(1.05873), .decimal = LSC_F2I_F(1.05873)}, {.integer = LSC_F2I_I(1.05786), .decimal = LSC_F2I_F(1.05786)}, {.integer = LSC_F2I_I(1.07311), .decimal = LSC_F2I_F(1.07311)}, {.integer = LSC_F2I_I(1.10608), .decimal = LSC_F2I_F(1.10608)}, {.integer = LSC_F2I_I(1.16076), .decimal = LSC_F2I_F(1.16076)}, {.integer = LSC_F2I_I(1.23093), .decimal = LSC_F2I_F(1.23093)}, {.integer = LSC_F2I_I(1.32376), .decimal = LSC_F2I_F(1.32376)}, {.integer = LSC_F2I_I(1.43783), .decimal = LSC_F2I_F(1.43783)}, {.integer = LSC_F2I_I(1.57734), .decimal = LSC_F2I_F(1.57734)}, {.integer = LSC_F2I_I(1.73549), .decimal = LSC_F2I_F(1.73549)}, {.integer = LSC_F2I_I(1.91685), .decimal = LSC_F2I_F(1.91685)}, {.integer = LSC_F2I_I(1.91685), .decimal = LSC_F2I_F(1.91685)},
    {.integer = LSC_F2I_I(2.00128), .decimal = LSC_F2I_F(2.00128)}, {.integer = LSC_F2I_I(1.81284), .decimal = LSC_F2I_F(1.81284)}, {.integer = LSC_F2I_I(1.65066), .decimal = LSC_F2I_F(1.65066)}, {.integer = LSC_F2I_I(1.50817), .decimal = LSC_F2I_F(1.50817)}, {.integer = LSC_F2I_I(1.39106), .decimal = LSC_F2I_F(1.39106)}, {.integer = LSC_F2I_I(1.29510), .decimal = LSC_F2I_F(1.29510)}, {.integer = LSC_F2I_I(1.22332), .decimal = LSC_F2I_F(1.22332)}, {.integer = LSC_F2I_I(1.17059), .decimal = LSC_F2I_F(1.17059)}, {.integer = LSC_F2I_I(1.13339), .decimal = LSC_F2I_F(1.13339)}, {.integer = LSC_F2I_I(1.11819), .decimal = LSC_F2I_F(1.11819)}, {.integer = LSC_F2I_I(1.11755), .decimal = LSC_F2I_F(1.11755)}, {.integer = LSC_F2I_I(1.13733), .decimal = LSC_F2I_F(1.13733)}, {.integer = LSC_F2I_I(1.17077), .decimal = LSC_F2I_F(1.17077)}, {.integer = LSC_F2I_I(1.22597), .decimal = LSC_F2I_F(1.22597)}, {.integer = LSC_F2I_I(1.29983), .decimal = LSC_F2I_F(1.29983)}, {.integer = LSC_F2I_I(1.39491), .decimal = LSC_F2I_F(1.39491)}, {.integer = LSC_F2I_I(1.51202), .decimal = LSC_F2I_F(1.51202)}, {.integer = LSC_F2I_I(1.64868), .decimal = LSC_F2I_F(1.64868)}, {.integer = LSC_F2I_I(1.80780), .decimal = LSC_F2I_F(1.80780)}, {.integer = LSC_F2I_I(1.98926), .decimal = LSC_F2I_F(1.98926)}, {.integer = LSC_F2I_I(1.98926), .decimal = LSC_F2I_F(1.98926)},
    {.integer = LSC_F2I_I(2.10219), .decimal = LSC_F2I_F(2.10219)}, {.integer = LSC_F2I_I(1.91088), .decimal = LSC_F2I_F(1.91088)}, {.integer = LSC_F2I_I(1.74314), .decimal = LSC_F2I_F(1.74314)}, {.integer = LSC_F2I_I(1.60055), .decimal = LSC_F2I_F(1.60055)}, {.integer = LSC_F2I_I(1.48157), .decimal = LSC_F2I_F(1.48157)}, {.integer = LSC_F2I_I(1.38567), .decimal = LSC_F2I_F(1.38567)}, {.integer = LSC_F2I_I(1.30976), .decimal = LSC_F2I_F(1.30976)}, {.integer = LSC_F2I_I(1.25627), .decimal = LSC_F2I_F(1.25627)}, {.integer = LSC_F2I_I(1.22129), .decimal = LSC_F2I_F(1.22129)}, {.integer = LSC_F2I_I(1.20263), .decimal = LSC_F2I_F(1.20263)}, {.integer = LSC_F2I_I(1.20116), .decimal = LSC_F2I_F(1.20116)}, {.integer = LSC_F2I_I(1.22184), .decimal = LSC_F2I_F(1.22184)}, {.integer = LSC_F2I_I(1.25899), .decimal = LSC_F2I_F(1.25899)}, {.integer = LSC_F2I_I(1.31496), .decimal = LSC_F2I_F(1.31496)}, {.integer = LSC_F2I_I(1.39211), .decimal = LSC_F2I_F(1.39211)}, {.integer = LSC_F2I_I(1.48832), .decimal = LSC_F2I_F(1.48832)}, {.integer = LSC_F2I_I(1.60810), .decimal = LSC_F2I_F(1.60810)}, {.integer = LSC_F2I_I(1.74668), .decimal = LSC_F2I_F(1.74668)}, {.integer = LSC_F2I_I(1.90468), .decimal = LSC_F2I_F(1.90468)}, {.integer = LSC_F2I_I(2.08417), .decimal = LSC_F2I_F(2.08417)}, {.integer = LSC_F2I_I(2.08417), .decimal = LSC_F2I_F(2.08417)},
    {.integer = LSC_F2I_I(2.18080), .decimal = LSC_F2I_F(2.18080)}, {.integer = LSC_F2I_I(1.98516), .decimal = LSC_F2I_F(1.98516)}, {.integer = LSC_F2I_I(1.82032), .decimal = LSC_F2I_F(1.82032)}, {.integer = LSC_F2I_I(1.67119), .decimal = LSC_F2I_F(1.67119)}, {.integer = LSC_F2I_I(1.55114), .decimal = LSC_F2I_F(1.55114)}, {.integer = LSC_F2I_I(1.45031), .decimal = LSC_F2I_F(1.45031)}, {.integer = LSC_F2I_I(1.37806), .decimal = LSC_F2I_F(1.37806)}, {.integer = LSC_F2I_I(1.32389), .decimal = LSC_F2I_F(1.32389)}, {.integer = LSC_F2I_I(1.28768), .decimal = LSC_F2I_F(1.28768)}, {.integer = LSC_F2I_I(1.26907), .decimal = LSC_F2I_F(1.26907)}, {.integer = LSC_F2I_I(1.26799), .decimal = LSC_F2I_F(1.26799)}, {.integer = LSC_F2I_I(1.28866), .decimal = LSC_F2I_F(1.28866)}, {.integer = LSC_F2I_I(1.32449), .decimal = LSC_F2I_F(1.32449)}, {.integer = LSC_F2I_I(1.38400), .decimal = LSC_F2I_F(1.38400)}, {.integer = LSC_F2I_I(1.46297), .decimal = LSC_F2I_F(1.46297)}, {.integer = LSC_F2I_I(1.55833), .decimal = LSC_F2I_F(1.55833)}, {.integer = LSC_F2I_I(1.67547), .decimal = LSC_F2I_F(1.67547)}, {.integer = LSC_F2I_I(1.81259), .decimal = LSC_F2I_F(1.81259)}, {.integer = LSC_F2I_I(1.97320), .decimal = LSC_F2I_F(1.97320)}, {.integer = LSC_F2I_I(2.15714), .decimal = LSC_F2I_F(2.15714)}, {.integer = LSC_F2I_I(2.15714), .decimal = LSC_F2I_F(2.15714)},
    {.integer = LSC_F2I_I(2.18080), .decimal = LSC_F2I_F(2.18080)}, {.integer = LSC_F2I_I(1.98516), .decimal = LSC_F2I_F(1.98516)}, {.integer = LSC_F2I_I(1.82032), .decimal = LSC_F2I_F(1.82032)}, {.integer = LSC_F2I_I(1.67119), .decimal = LSC_F2I_F(1.67119)}, {.integer = LSC_F2I_I(1.55114), .decimal = LSC_F2I_F(1.55114)}, {.integer = LSC_F2I_I(1.45031), .decimal = LSC_F2I_F(1.45031)}, {.integer = LSC_F2I_I(1.37806), .decimal = LSC_F2I_F(1.37806)}, {.integer = LSC_F2I_I(1.32389), .decimal = LSC_F2I_F(1.32389)}, {.integer = LSC_F2I_I(1.28768), .decimal = LSC_F2I_F(1.28768)}, {.integer = LSC_F2I_I(1.26907), .decimal = LSC_F2I_F(1.26907)}, {.integer = LSC_F2I_I(1.26799), .decimal = LSC_F2I_F(1.26799)}, {.integer = LSC_F2I_I(1.28866), .decimal = LSC_F2I_F(1.28866)}, {.integer = LSC_F2I_I(1.32449), .decimal = LSC_F2I_F(1.32449)}, {.integer = LSC_F2I_I(1.38400), .decimal = LSC_F2I_F(1.38400)}, {.integer = LSC_F2I_I(1.46297), .decimal = LSC_F2I_F(1.46297)}, {.integer = LSC_F2I_I(1.55833), .decimal = LSC_F2I_F(1.55833)}, {.integer = LSC_F2I_I(1.67547), .decimal = LSC_F2I_F(1.67547)}, {.integer = LSC_F2I_I(1.81259), .decimal = LSC_F2I_F(1.81259)}, {.integer = LSC_F2I_I(1.97320), .decimal = LSC_F2I_F(1.97320)}, {.integer = LSC_F2I_I(2.15714), .decimal = LSC_F2I_F(2.15714)}, {.integer = LSC_F2I_I(2.15714), .decimal = LSC_F2I_F(2.15714)}
};

isp_lsc_gain_t gb_gain[13 * 21] = {
    {.integer = LSC_F2I_I(2.15978), .decimal = LSC_F2I_F(2.15978)}, {.integer = LSC_F2I_I(1.97189), .decimal = LSC_F2I_F(1.97189)}, {.integer = LSC_F2I_I(1.80124), .decimal = LSC_F2I_F(1.80124)}, {.integer = LSC_F2I_I(1.65887), .decimal = LSC_F2I_F(1.65887)}, {.integer = LSC_F2I_I(1.53894), .decimal = LSC_F2I_F(1.53894)}, {.integer = LSC_F2I_I(1.44064), .decimal = LSC_F2I_F(1.44064)}, {.integer = LSC_F2I_I(1.36581), .decimal = LSC_F2I_F(1.36581)}, {.integer = LSC_F2I_I(1.31207), .decimal = LSC_F2I_F(1.31207)}, {.integer = LSC_F2I_I(1.30259), .decimal = LSC_F2I_F(1.30259)}, {.integer = LSC_F2I_I(1.27946), .decimal = LSC_F2I_F(1.27946)}, {.integer = LSC_F2I_I(1.25355), .decimal = LSC_F2I_F(1.25355)}, {.integer = LSC_F2I_I(1.27024), .decimal = LSC_F2I_F(1.27024)}, {.integer = LSC_F2I_I(1.30701), .decimal = LSC_F2I_F(1.30701)}, {.integer = LSC_F2I_I(1.36744), .decimal = LSC_F2I_F(1.36744)}, {.integer = LSC_F2I_I(1.44077), .decimal = LSC_F2I_F(1.44077)}, {.integer = LSC_F2I_I(1.53187), .decimal = LSC_F2I_F(1.53187)}, {.integer = LSC_F2I_I(1.64642), .decimal = LSC_F2I_F(1.64642)}, {.integer = LSC_F2I_I(1.78143), .decimal = LSC_F2I_F(1.78143)}, {.integer = LSC_F2I_I(1.94065), .decimal = LSC_F2I_F(1.94065)}, {.integer = LSC_F2I_I(2.11596), .decimal = LSC_F2I_F(2.11596)}, {.integer = LSC_F2I_I(2.11596), .decimal = LSC_F2I_F(2.11596)},
    {.integer = LSC_F2I_I(2.05558), .decimal = LSC_F2I_F(2.05558)}, {.integer = LSC_F2I_I(1.86338), .decimal = LSC_F2I_F(1.86338)}, {.integer = LSC_F2I_I(1.69894), .decimal = LSC_F2I_F(1.69894)}, {.integer = LSC_F2I_I(1.55594), .decimal = LSC_F2I_F(1.55594)}, {.integer = LSC_F2I_I(1.43614), .decimal = LSC_F2I_F(1.43614)}, {.integer = LSC_F2I_I(1.34068), .decimal = LSC_F2I_F(1.34068)}, {.integer = LSC_F2I_I(1.26814), .decimal = LSC_F2I_F(1.26814)}, {.integer = LSC_F2I_I(1.22022), .decimal = LSC_F2I_F(1.22022)}, {.integer = LSC_F2I_I(1.20150), .decimal = LSC_F2I_F(1.20150)}, {.integer = LSC_F2I_I(1.18485), .decimal = LSC_F2I_F(1.18485)}, {.integer = LSC_F2I_I(1.16066), .decimal = LSC_F2I_F(1.16066)}, {.integer = LSC_F2I_I(1.17417), .decimal = LSC_F2I_F(1.17417)}, {.integer = LSC_F2I_I(1.20972), .decimal = LSC_F2I_F(1.20972)}, {.integer = LSC_F2I_I(1.26350), .decimal = LSC_F2I_F(1.26350)}, {.integer = LSC_F2I_I(1.33621), .decimal = LSC_F2I_F(1.33621)}, {.integer = LSC_F2I_I(1.42986), .decimal = LSC_F2I_F(1.42986)}, {.integer = LSC_F2I_I(1.54297), .decimal = LSC_F2I_F(1.54297)}, {.integer = LSC_F2I_I(1.67931), .decimal = LSC_F2I_F(1.67931)}, {.integer = LSC_F2I_I(1.83667), .decimal = LSC_F2I_F(1.83667)}, {.integer = LSC_F2I_I(2.00885), .decimal = LSC_F2I_F(2.00885)}, {.integer = LSC_F2I_I(2.00885), .decimal = LSC_F2I_F(2.00885)},
    {.integer = LSC_F2I_I(1.97316), .decimal = LSC_F2I_F(1.97316)}, {.integer = LSC_F2I_I(1.78384), .decimal = LSC_F2I_F(1.78384)}, {.integer = LSC_F2I_I(1.61712), .decimal = LSC_F2I_F(1.61712)}, {.integer = LSC_F2I_I(1.47826), .decimal = LSC_F2I_F(1.47826)}, {.integer = LSC_F2I_I(1.36074), .decimal = LSC_F2I_F(1.36074)}, {.integer = LSC_F2I_I(1.26688), .decimal = LSC_F2I_F(1.26688)}, {.integer = LSC_F2I_I(1.19478), .decimal = LSC_F2I_F(1.19478)}, {.integer = LSC_F2I_I(1.14038), .decimal = LSC_F2I_F(1.14038)}, {.integer = LSC_F2I_I(1.11348), .decimal = LSC_F2I_F(1.11348)}, {.integer = LSC_F2I_I(1.09558), .decimal = LSC_F2I_F(1.09558)}, {.integer = LSC_F2I_I(1.08864), .decimal = LSC_F2I_F(1.08864)}, {.integer = LSC_F2I_I(1.10360), .decimal = LSC_F2I_F(1.10360)}, {.integer = LSC_F2I_I(1.13628), .decimal = LSC_F2I_F(1.13628)}, {.integer = LSC_F2I_I(1.18651), .decimal = LSC_F2I_F(1.18651)}, {.integer = LSC_F2I_I(1.25653), .decimal = LSC_F2I_F(1.25653)}, {.integer = LSC_F2I_I(1.34868), .decimal = LSC_F2I_F(1.34868)}, {.integer = LSC_F2I_I(1.46377), .decimal = LSC_F2I_F(1.46377)}, {.integer = LSC_F2I_I(1.59555), .decimal = LSC_F2I_F(1.59555)}, {.integer = LSC_F2I_I(1.74935), .decimal = LSC_F2I_F(1.74935)}, {.integer = LSC_F2I_I(1.92803), .decimal = LSC_F2I_F(1.92803)}, {.integer = LSC_F2I_I(1.92803), .decimal = LSC_F2I_F(1.92803)},
    {.integer = LSC_F2I_I(1.91507), .decimal = LSC_F2I_F(1.91507)}, {.integer = LSC_F2I_I(1.72574), .decimal = LSC_F2I_F(1.72574)}, {.integer = LSC_F2I_I(1.55953), .decimal = LSC_F2I_F(1.55953)}, {.integer = LSC_F2I_I(1.41899), .decimal = LSC_F2I_F(1.41899)}, {.integer = LSC_F2I_I(1.30472), .decimal = LSC_F2I_F(1.30472)}, {.integer = LSC_F2I_I(1.21210), .decimal = LSC_F2I_F(1.21210)}, {.integer = LSC_F2I_I(1.14101), .decimal = LSC_F2I_F(1.14101)}, {.integer = LSC_F2I_I(1.09191), .decimal = LSC_F2I_F(1.09191)}, {.integer = LSC_F2I_I(1.05919), .decimal = LSC_F2I_F(1.05919)}, {.integer = LSC_F2I_I(1.04446), .decimal = LSC_F2I_F(1.04446)}, {.integer = LSC_F2I_I(1.04267), .decimal = LSC_F2I_F(1.04267)}, {.integer = LSC_F2I_I(1.05523), .decimal = LSC_F2I_F(1.05523)}, {.integer = LSC_F2I_I(1.08333), .decimal = LSC_F2I_F(1.08333)}, {.integer = LSC_F2I_I(1.13133), .decimal = LSC_F2I_F(1.13133)}, {.integer = LSC_F2I_I(1.20196), .decimal = LSC_F2I_F(1.20196)}, {.integer = LSC_F2I_I(1.29247), .decimal = LSC_F2I_F(1.29247)}, {.integer = LSC_F2I_I(1.40470), .decimal = LSC_F2I_F(1.40470)}, {.integer = LSC_F2I_I(1.53950), .decimal = LSC_F2I_F(1.53950)}, {.integer = LSC_F2I_I(1.69325), .decimal = LSC_F2I_F(1.69325)}, {.integer = LSC_F2I_I(1.87447), .decimal = LSC_F2I_F(1.87447)}, {.integer = LSC_F2I_I(1.87447), .decimal = LSC_F2I_F(1.87447)},
    {.integer = LSC_F2I_I(1.88452), .decimal = LSC_F2I_F(1.88452)}, {.integer = LSC_F2I_I(1.69407), .decimal = LSC_F2I_F(1.69407)}, {.integer = LSC_F2I_I(1.52496), .decimal = LSC_F2I_F(1.52496)}, {.integer = LSC_F2I_I(1.38575), .decimal = LSC_F2I_F(1.38575)}, {.integer = LSC_F2I_I(1.27092), .decimal = LSC_F2I_F(1.27092)}, {.integer = LSC_F2I_I(1.17985), .decimal = LSC_F2I_F(1.17985)}, {.integer = LSC_F2I_I(1.11062), .decimal = LSC_F2I_F(1.11062)}, {.integer = LSC_F2I_I(1.06045), .decimal = LSC_F2I_F(1.06045)}, {.integer = LSC_F2I_I(1.02945), .decimal = LSC_F2I_F(1.02945)}, {.integer = LSC_F2I_I(1.01473), .decimal = LSC_F2I_F(1.01473)}, {.integer = LSC_F2I_I(1.01180), .decimal = LSC_F2I_F(1.01180)}, {.integer = LSC_F2I_I(1.02466), .decimal = LSC_F2I_F(1.02466)}, {.integer = LSC_F2I_I(1.05068), .decimal = LSC_F2I_F(1.05068)}, {.integer = LSC_F2I_I(1.09862), .decimal = LSC_F2I_F(1.09862)}, {.integer = LSC_F2I_I(1.16650), .decimal = LSC_F2I_F(1.16650)}, {.integer = LSC_F2I_I(1.25712), .decimal = LSC_F2I_F(1.25712)}, {.integer = LSC_F2I_I(1.37155), .decimal = LSC_F2I_F(1.37155)}, {.integer = LSC_F2I_I(1.50361), .decimal = LSC_F2I_F(1.50361)}, {.integer = LSC_F2I_I(1.65937), .decimal = LSC_F2I_F(1.65937)}, {.integer = LSC_F2I_I(1.83245), .decimal = LSC_F2I_F(1.83245)}, {.integer = LSC_F2I_I(1.83245), .decimal = LSC_F2I_F(1.83245)},
    {.integer = LSC_F2I_I(1.87204), .decimal = LSC_F2I_F(1.87204)}, {.integer = LSC_F2I_I(1.68040), .decimal = LSC_F2I_F(1.68040)}, {.integer = LSC_F2I_I(1.51584), .decimal = LSC_F2I_F(1.51584)}, {.integer = LSC_F2I_I(1.37353), .decimal = LSC_F2I_F(1.37353)}, {.integer = LSC_F2I_I(1.25849), .decimal = LSC_F2I_F(1.25849)}, {.integer = LSC_F2I_I(1.16639), .decimal = LSC_F2I_F(1.16639)}, {.integer = LSC_F2I_I(1.09722), .decimal = LSC_F2I_F(1.09722)}, {.integer = LSC_F2I_I(1.04600), .decimal = LSC_F2I_F(1.04600)}, {.integer = LSC_F2I_I(1.01550), .decimal = LSC_F2I_F(1.01550)}, {.integer = LSC_F2I_I(0.99895), .decimal = LSC_F2I_F(0.99895)}, {.integer = LSC_F2I_I(0.99838), .decimal = LSC_F2I_F(0.99838)}, {.integer = LSC_F2I_I(1.00864), .decimal = LSC_F2I_F(1.00864)}, {.integer = LSC_F2I_I(1.03755), .decimal = LSC_F2I_F(1.03755)}, {.integer = LSC_F2I_I(1.08601), .decimal = LSC_F2I_F(1.08601)}, {.integer = LSC_F2I_I(1.15284), .decimal = LSC_F2I_F(1.15284)}, {.integer = LSC_F2I_I(1.24384), .decimal = LSC_F2I_F(1.24384)}, {.integer = LSC_F2I_I(1.35705), .decimal = LSC_F2I_F(1.35705)}, {.integer = LSC_F2I_I(1.48966), .decimal = LSC_F2I_F(1.48966)}, {.integer = LSC_F2I_I(1.64384), .decimal = LSC_F2I_F(1.64384)}, {.integer = LSC_F2I_I(1.82050), .decimal = LSC_F2I_F(1.82050)}, {.integer = LSC_F2I_I(1.82050), .decimal = LSC_F2I_F(1.82050)},
    {.integer = LSC_F2I_I(1.88342), .decimal = LSC_F2I_F(1.88342)}, {.integer = LSC_F2I_I(1.69234), .decimal = LSC_F2I_F(1.69234)}, {.integer = LSC_F2I_I(1.52608), .decimal = LSC_F2I_F(1.52608)}, {.integer = LSC_F2I_I(1.38207), .decimal = LSC_F2I_F(1.38207)}, {.integer = LSC_F2I_I(1.26464), .decimal = LSC_F2I_F(1.26464)}, {.integer = LSC_F2I_I(1.17487), .decimal = LSC_F2I_F(1.17487)}, {.integer = LSC_F2I_I(1.10250), .decimal = LSC_F2I_F(1.10250)}, {.integer = LSC_F2I_I(1.05228), .decimal = LSC_F2I_F(1.05228)}, {.integer = LSC_F2I_I(1.01954), .decimal = LSC_F2I_F(1.01954)}, {.integer = LSC_F2I_I(1.00156), .decimal = LSC_F2I_F(1.00156)}, {.integer = LSC_F2I_I(1.00000), .decimal = LSC_F2I_F(1.00000)}, {.integer = LSC_F2I_I(1.01247), .decimal = LSC_F2I_F(1.01247)}, {.integer = LSC_F2I_I(1.04245), .decimal = LSC_F2I_F(1.04245)}, {.integer = LSC_F2I_I(1.08968), .decimal = LSC_F2I_F(1.08968)}, {.integer = LSC_F2I_I(1.15809), .decimal = LSC_F2I_F(1.15809)}, {.integer = LSC_F2I_I(1.24920), .decimal = LSC_F2I_F(1.24920)}, {.integer = LSC_F2I_I(1.36300), .decimal = LSC_F2I_F(1.36300)}, {.integer = LSC_F2I_I(1.49797), .decimal = LSC_F2I_F(1.49797)}, {.integer = LSC_F2I_I(1.65506), .decimal = LSC_F2I_F(1.65506)}, {.integer = LSC_F2I_I(1.82986), .decimal = LSC_F2I_F(1.82986)}, {.integer = LSC_F2I_I(1.82986), .decimal = LSC_F2I_F(1.82986)},
    {.integer = LSC_F2I_I(1.91768), .decimal = LSC_F2I_F(1.91768)}, {.integer = LSC_F2I_I(1.72891), .decimal = LSC_F2I_F(1.72891)}, {.integer = LSC_F2I_I(1.55833), .decimal = LSC_F2I_F(1.55833)}, {.integer = LSC_F2I_I(1.41595), .decimal = LSC_F2I_F(1.41595)}, {.integer = LSC_F2I_I(1.29714), .decimal = LSC_F2I_F(1.29714)}, {.integer = LSC_F2I_I(1.20305), .decimal = LSC_F2I_F(1.20305)}, {.integer = LSC_F2I_I(1.12818), .decimal = LSC_F2I_F(1.12818)}, {.integer = LSC_F2I_I(1.07592), .decimal = LSC_F2I_F(1.07592)}, {.integer = LSC_F2I_I(1.04221), .decimal = LSC_F2I_F(1.04221)}, {.integer = LSC_F2I_I(1.02384), .decimal = LSC_F2I_F(1.02384)}, {.integer = LSC_F2I_I(1.02180), .decimal = LSC_F2I_F(1.02180)}, {.integer = LSC_F2I_I(1.03606), .decimal = LSC_F2I_F(1.03606)}, {.integer = LSC_F2I_I(1.06656), .decimal = LSC_F2I_F(1.06656)}, {.integer = LSC_F2I_I(1.11630), .decimal = LSC_F2I_F(1.11630)}, {.integer = LSC_F2I_I(1.18814), .decimal = LSC_F2I_F(1.18814)}, {.integer = LSC_F2I_I(1.28003), .decimal = LSC_F2I_F(1.28003)}, {.integer = LSC_F2I_I(1.39635), .decimal = LSC_F2I_F(1.39635)}, {.integer = LSC_F2I_I(1.52843), .decimal = LSC_F2I_F(1.52843)}, {.integer = LSC_F2I_I(1.68555), .decimal = LSC_F2I_F(1.68555)}, {.integer = LSC_F2I_I(1.86150), .decimal = LSC_F2I_F(1.86150)}, {.integer = LSC_F2I_I(1.86150), .decimal = LSC_F2I_F(1.86150)},
    {.integer = LSC_F2I_I(1.98393), .decimal = LSC_F2I_F(1.98393)}, {.integer = LSC_F2I_I(1.78549), .decimal = LSC_F2I_F(1.78549)}, {.integer = LSC_F2I_I(1.61483), .decimal = LSC_F2I_F(1.61483)}, {.integer = LSC_F2I_I(1.47061), .decimal = LSC_F2I_F(1.47061)}, {.integer = LSC_F2I_I(1.34718), .decimal = LSC_F2I_F(1.34718)}, {.integer = LSC_F2I_I(1.25219), .decimal = LSC_F2I_F(1.25219)}, {.integer = LSC_F2I_I(1.17609), .decimal = LSC_F2I_F(1.17609)}, {.integer = LSC_F2I_I(1.12043), .decimal = LSC_F2I_F(1.12043)}, {.integer = LSC_F2I_I(1.08361), .decimal = LSC_F2I_F(1.08361)}, {.integer = LSC_F2I_I(1.06531), .decimal = LSC_F2I_F(1.06531)}, {.integer = LSC_F2I_I(1.06280), .decimal = LSC_F2I_F(1.06280)}, {.integer = LSC_F2I_I(1.07790), .decimal = LSC_F2I_F(1.07790)}, {.integer = LSC_F2I_I(1.11017), .decimal = LSC_F2I_F(1.11017)}, {.integer = LSC_F2I_I(1.16447), .decimal = LSC_F2I_F(1.16447)}, {.integer = LSC_F2I_I(1.23833), .decimal = LSC_F2I_F(1.23833)}, {.integer = LSC_F2I_I(1.33309), .decimal = LSC_F2I_F(1.33309)}, {.integer = LSC_F2I_I(1.44876), .decimal = LSC_F2I_F(1.44876)}, {.integer = LSC_F2I_I(1.58371), .decimal = LSC_F2I_F(1.58371)}, {.integer = LSC_F2I_I(1.74037), .decimal = LSC_F2I_F(1.74037)}, {.integer = LSC_F2I_I(1.91541), .decimal = LSC_F2I_F(1.91541)}, {.integer = LSC_F2I_I(1.91541), .decimal = LSC_F2I_F(1.91541)},
    {.integer = LSC_F2I_I(2.06778), .decimal = LSC_F2I_F(2.06778)}, {.integer = LSC_F2I_I(1.86914), .decimal = LSC_F2I_F(1.86914)}, {.integer = LSC_F2I_I(1.69606), .decimal = LSC_F2I_F(1.69606)}, {.integer = LSC_F2I_I(1.54766), .decimal = LSC_F2I_F(1.54766)}, {.integer = LSC_F2I_I(1.42258), .decimal = LSC_F2I_F(1.42258)}, {.integer = LSC_F2I_I(1.32252), .decimal = LSC_F2I_F(1.32252)}, {.integer = LSC_F2I_I(1.24551), .decimal = LSC_F2I_F(1.24551)}, {.integer = LSC_F2I_I(1.18737), .decimal = LSC_F2I_F(1.18737)}, {.integer = LSC_F2I_I(1.14813), .decimal = LSC_F2I_F(1.14813)}, {.integer = LSC_F2I_I(1.13046), .decimal = LSC_F2I_F(1.13046)}, {.integer = LSC_F2I_I(1.12903), .decimal = LSC_F2I_F(1.12903)}, {.integer = LSC_F2I_I(1.14398), .decimal = LSC_F2I_F(1.14398)}, {.integer = LSC_F2I_I(1.17913), .decimal = LSC_F2I_F(1.17913)}, {.integer = LSC_F2I_I(1.23432), .decimal = LSC_F2I_F(1.23432)}, {.integer = LSC_F2I_I(1.31038), .decimal = LSC_F2I_F(1.31038)}, {.integer = LSC_F2I_I(1.40645), .decimal = LSC_F2I_F(1.40645)}, {.integer = LSC_F2I_I(1.52395), .decimal = LSC_F2I_F(1.52395)}, {.integer = LSC_F2I_I(1.66060), .decimal = LSC_F2I_F(1.66060)}, {.integer = LSC_F2I_I(1.81564), .decimal = LSC_F2I_F(1.81564)}, {.integer = LSC_F2I_I(1.99376), .decimal = LSC_F2I_F(1.99376)}, {.integer = LSC_F2I_I(1.99376), .decimal = LSC_F2I_F(1.99376)},
    {.integer = LSC_F2I_I(2.17562), .decimal = LSC_F2I_F(2.17562)}, {.integer = LSC_F2I_I(1.97793), .decimal = LSC_F2I_F(1.97793)}, {.integer = LSC_F2I_I(1.80168), .decimal = LSC_F2I_F(1.80168)}, {.integer = LSC_F2I_I(1.65133), .decimal = LSC_F2I_F(1.65133)}, {.integer = LSC_F2I_I(1.52262), .decimal = LSC_F2I_F(1.52262)}, {.integer = LSC_F2I_I(1.41904), .decimal = LSC_F2I_F(1.41904)}, {.integer = LSC_F2I_I(1.33791), .decimal = LSC_F2I_F(1.33791)}, {.integer = LSC_F2I_I(1.28012), .decimal = LSC_F2I_F(1.28012)}, {.integer = LSC_F2I_I(1.24124), .decimal = LSC_F2I_F(1.24124)}, {.integer = LSC_F2I_I(1.21717), .decimal = LSC_F2I_F(1.21717)}, {.integer = LSC_F2I_I(1.21696), .decimal = LSC_F2I_F(1.21696)}, {.integer = LSC_F2I_I(1.23330), .decimal = LSC_F2I_F(1.23330)}, {.integer = LSC_F2I_I(1.27101), .decimal = LSC_F2I_F(1.27101)}, {.integer = LSC_F2I_I(1.32842), .decimal = LSC_F2I_F(1.32842)}, {.integer = LSC_F2I_I(1.40365), .decimal = LSC_F2I_F(1.40365)}, {.integer = LSC_F2I_I(1.50200), .decimal = LSC_F2I_F(1.50200)}, {.integer = LSC_F2I_I(1.61882), .decimal = LSC_F2I_F(1.61882)}, {.integer = LSC_F2I_I(1.75752), .decimal = LSC_F2I_F(1.75752)}, {.integer = LSC_F2I_I(1.91239), .decimal = LSC_F2I_F(1.91239)}, {.integer = LSC_F2I_I(2.08934), .decimal = LSC_F2I_F(2.08934)}, {.integer = LSC_F2I_I(2.08934), .decimal = LSC_F2I_F(2.08934)},
    {.integer = LSC_F2I_I(2.26027), .decimal = LSC_F2I_F(2.26027)}, {.integer = LSC_F2I_I(2.06121), .decimal = LSC_F2I_F(2.06121)}, {.integer = LSC_F2I_I(1.88481), .decimal = LSC_F2I_F(1.88481)}, {.integer = LSC_F2I_I(1.73158), .decimal = LSC_F2I_F(1.73158)}, {.integer = LSC_F2I_I(1.59653), .decimal = LSC_F2I_F(1.59653)}, {.integer = LSC_F2I_I(1.49606), .decimal = LSC_F2I_F(1.49606)}, {.integer = LSC_F2I_I(1.41248), .decimal = LSC_F2I_F(1.41248)}, {.integer = LSC_F2I_I(1.35287), .decimal = LSC_F2I_F(1.35287)}, {.integer = LSC_F2I_I(1.31298), .decimal = LSC_F2I_F(1.31298)}, {.integer = LSC_F2I_I(1.28818), .decimal = LSC_F2I_F(1.28818)}, {.integer = LSC_F2I_I(1.28648), .decimal = LSC_F2I_F(1.28648)}, {.integer = LSC_F2I_I(1.30598), .decimal = LSC_F2I_F(1.30598)}, {.integer = LSC_F2I_I(1.34008), .decimal = LSC_F2I_F(1.34008)}, {.integer = LSC_F2I_I(1.39739), .decimal = LSC_F2I_F(1.39739)}, {.integer = LSC_F2I_I(1.47854), .decimal = LSC_F2I_F(1.47854)}, {.integer = LSC_F2I_I(1.57353), .decimal = LSC_F2I_F(1.57353)}, {.integer = LSC_F2I_I(1.69627), .decimal = LSC_F2I_F(1.69627)}, {.integer = LSC_F2I_I(1.83594), .decimal = LSC_F2I_F(1.83594)}, {.integer = LSC_F2I_I(1.98203), .decimal = LSC_F2I_F(1.98203)}, {.integer = LSC_F2I_I(2.16250), .decimal = LSC_F2I_F(2.16250)}, {.integer = LSC_F2I_I(2.16250), .decimal = LSC_F2I_F(2.16250)},
    {.integer = LSC_F2I_I(2.26027), .decimal = LSC_F2I_F(2.26027)}, {.integer = LSC_F2I_I(2.06121), .decimal = LSC_F2I_F(2.06121)}, {.integer = LSC_F2I_I(1.88481), .decimal = LSC_F2I_F(1.88481)}, {.integer = LSC_F2I_I(1.73158), .decimal = LSC_F2I_F(1.73158)}, {.integer = LSC_F2I_I(1.59653), .decimal = LSC_F2I_F(1.59653)}, {.integer = LSC_F2I_I(1.49606), .decimal = LSC_F2I_F(1.49606)}, {.integer = LSC_F2I_I(1.41248), .decimal = LSC_F2I_F(1.41248)}, {.integer = LSC_F2I_I(1.35287), .decimal = LSC_F2I_F(1.35287)}, {.integer = LSC_F2I_I(1.31298), .decimal = LSC_F2I_F(1.31298)}, {.integer = LSC_F2I_I(1.28818), .decimal = LSC_F2I_F(1.28818)}, {.integer = LSC_F2I_I(1.28648), .decimal = LSC_F2I_F(1.28648)}, {.integer = LSC_F2I_I(1.30598), .decimal = LSC_F2I_F(1.30598)}, {.integer = LSC_F2I_I(1.34008), .decimal = LSC_F2I_F(1.34008)}, {.integer = LSC_F2I_I(1.39739), .decimal = LSC_F2I_F(1.39739)}, {.integer = LSC_F2I_I(1.47854), .decimal = LSC_F2I_F(1.47854)}, {.integer = LSC_F2I_I(1.57353), .decimal = LSC_F2I_F(1.57353)}, {.integer = LSC_F2I_I(1.69627), .decimal = LSC_F2I_F(1.69627)}, {.integer = LSC_F2I_I(1.83594), .decimal = LSC_F2I_F(1.83594)}, {.integer = LSC_F2I_I(1.98203), .decimal = LSC_F2I_F(1.98203)}, {.integer = LSC_F2I_I(2.16250), .decimal = LSC_F2I_F(2.16250)}, {.integer = LSC_F2I_I(2.16250), .decimal = LSC_F2I_F(2.16250)}
};

isp_lsc_gain_t b_gain[13 * 21] = {
    {.integer = LSC_F2I_I(1.99894), .decimal = LSC_F2I_F(1.99894)}, {.integer = LSC_F2I_I(1.83386), .decimal = LSC_F2I_F(1.83386)}, {.integer = LSC_F2I_I(1.68833), .decimal = LSC_F2I_F(1.68833)}, {.integer = LSC_F2I_I(1.56968), .decimal = LSC_F2I_F(1.56968)}, {.integer = LSC_F2I_I(1.46855), .decimal = LSC_F2I_F(1.46855)}, {.integer = LSC_F2I_I(1.38401), .decimal = LSC_F2I_F(1.38401)}, {.integer = LSC_F2I_I(1.32092), .decimal = LSC_F2I_F(1.32092)}, {.integer = LSC_F2I_I(1.27735), .decimal = LSC_F2I_F(1.27735)}, {.integer = LSC_F2I_I(1.27406), .decimal = LSC_F2I_F(1.27406)}, {.integer = LSC_F2I_I(1.25467), .decimal = LSC_F2I_F(1.25467)}, {.integer = LSC_F2I_I(1.22734), .decimal = LSC_F2I_F(1.22734)}, {.integer = LSC_F2I_I(1.23774), .decimal = LSC_F2I_F(1.23774)}, {.integer = LSC_F2I_I(1.26380), .decimal = LSC_F2I_F(1.26380)}, {.integer = LSC_F2I_I(1.31397), .decimal = LSC_F2I_F(1.31397)}, {.integer = LSC_F2I_I(1.37231), .decimal = LSC_F2I_F(1.37231)}, {.integer = LSC_F2I_I(1.44421), .decimal = LSC_F2I_F(1.44421)}, {.integer = LSC_F2I_I(1.53765), .decimal = LSC_F2I_F(1.53765)}, {.integer = LSC_F2I_I(1.64804), .decimal = LSC_F2I_F(1.64804)}, {.integer = LSC_F2I_I(1.77171), .decimal = LSC_F2I_F(1.77171)}, {.integer = LSC_F2I_I(1.91568), .decimal = LSC_F2I_F(1.91568)}, {.integer = LSC_F2I_I(1.91568), .decimal = LSC_F2I_F(1.91568)},
    {.integer = LSC_F2I_I(1.91069), .decimal = LSC_F2I_F(1.91069)}, {.integer = LSC_F2I_I(1.74477), .decimal = LSC_F2I_F(1.74477)}, {.integer = LSC_F2I_I(1.59627), .decimal = LSC_F2I_F(1.59627)}, {.integer = LSC_F2I_I(1.48160), .decimal = LSC_F2I_F(1.48160)}, {.integer = LSC_F2I_I(1.37893), .decimal = LSC_F2I_F(1.37893)}, {.integer = LSC_F2I_I(1.29760), .decimal = LSC_F2I_F(1.29760)}, {.integer = LSC_F2I_I(1.23678), .decimal = LSC_F2I_F(1.23678)}, {.integer = LSC_F2I_I(1.19482), .decimal = LSC_F2I_F(1.19482)}, {.integer = LSC_F2I_I(1.18066), .decimal = LSC_F2I_F(1.18066)}, {.integer = LSC_F2I_I(1.16908), .decimal = LSC_F2I_F(1.16908)}, {.integer = LSC_F2I_I(1.14216), .decimal = LSC_F2I_F(1.14216)}, {.integer = LSC_F2I_I(1.15269), .decimal = LSC_F2I_F(1.15269)}, {.integer = LSC_F2I_I(1.17857), .decimal = LSC_F2I_F(1.17857)}, {.integer = LSC_F2I_I(1.22190), .decimal = LSC_F2I_F(1.22190)}, {.integer = LSC_F2I_I(1.28061), .decimal = LSC_F2I_F(1.28061)}, {.integer = LSC_F2I_I(1.35709), .decimal = LSC_F2I_F(1.35709)}, {.integer = LSC_F2I_I(1.45230), .decimal = LSC_F2I_F(1.45230)}, {.integer = LSC_F2I_I(1.56082), .decimal = LSC_F2I_F(1.56082)}, {.integer = LSC_F2I_I(1.68778), .decimal = LSC_F2I_F(1.68778)}, {.integer = LSC_F2I_I(1.82635), .decimal = LSC_F2I_F(1.82635)}, {.integer = LSC_F2I_I(1.82635), .decimal = LSC_F2I_F(1.82635)},
    {.integer = LSC_F2I_I(1.84456), .decimal = LSC_F2I_F(1.84456)}, {.integer = LSC_F2I_I(1.67629), .decimal = LSC_F2I_F(1.67629)}, {.integer = LSC_F2I_I(1.53053), .decimal = LSC_F2I_F(1.53053)}, {.integer = LSC_F2I_I(1.41233), .decimal = LSC_F2I_F(1.41233)}, {.integer = LSC_F2I_I(1.30949), .decimal = LSC_F2I_F(1.30949)}, {.integer = LSC_F2I_I(1.23048), .decimal = LSC_F2I_F(1.23048)}, {.integer = LSC_F2I_I(1.16926), .decimal = LSC_F2I_F(1.16926)}, {.integer = LSC_F2I_I(1.12394), .decimal = LSC_F2I_F(1.12394)}, {.integer = LSC_F2I_I(1.10275), .decimal = LSC_F2I_F(1.10275)}, {.integer = LSC_F2I_I(1.08486), .decimal = LSC_F2I_F(1.08486)}, {.integer = LSC_F2I_I(1.07985), .decimal = LSC_F2I_F(1.07985)}, {.integer = LSC_F2I_I(1.09347), .decimal = LSC_F2I_F(1.09347)}, {.integer = LSC_F2I_I(1.11500), .decimal = LSC_F2I_F(1.11500)}, {.integer = LSC_F2I_I(1.15706), .decimal = LSC_F2I_F(1.15706)}, {.integer = LSC_F2I_I(1.21283), .decimal = LSC_F2I_F(1.21283)}, {.integer = LSC_F2I_I(1.28721), .decimal = LSC_F2I_F(1.28721)}, {.integer = LSC_F2I_I(1.38659), .decimal = LSC_F2I_F(1.38659)}, {.integer = LSC_F2I_I(1.49449), .decimal = LSC_F2I_F(1.49449)}, {.integer = LSC_F2I_I(1.62031), .decimal = LSC_F2I_F(1.62031)}, {.integer = LSC_F2I_I(1.76469), .decimal = LSC_F2I_F(1.76469)}, {.integer = LSC_F2I_I(1.76469), .decimal = LSC_F2I_F(1.76469)},
    {.integer = LSC_F2I_I(1.79758), .decimal = LSC_F2I_F(1.79758)}, {.integer = LSC_F2I_I(1.62827), .decimal = LSC_F2I_F(1.62827)}, {.integer = LSC_F2I_I(1.48298), .decimal = LSC_F2I_F(1.48298)}, {.integer = LSC_F2I_I(1.36455), .decimal = LSC_F2I_F(1.36455)}, {.integer = LSC_F2I_I(1.26438), .decimal = LSC_F2I_F(1.26438)}, {.integer = LSC_F2I_I(1.18362), .decimal = LSC_F2I_F(1.18362)}, {.integer = LSC_F2I_I(1.12236), .decimal = LSC_F2I_F(1.12236)}, {.integer = LSC_F2I_I(1.07945), .decimal = LSC_F2I_F(1.07945)}, {.integer = LSC_F2I_I(1.05086), .decimal = LSC_F2I_F(1.05086)}, {.integer = LSC_F2I_I(1.03826), .decimal = LSC_F2I_F(1.03826)}, {.integer = LSC_F2I_I(1.03680), .decimal = LSC_F2I_F(1.03680)}, {.integer = LSC_F2I_I(1.04906), .decimal = LSC_F2I_F(1.04906)}, {.integer = LSC_F2I_I(1.07153), .decimal = LSC_F2I_F(1.07153)}, {.integer = LSC_F2I_I(1.11137), .decimal = LSC_F2I_F(1.11137)}, {.integer = LSC_F2I_I(1.16550), .decimal = LSC_F2I_F(1.16550)}, {.integer = LSC_F2I_I(1.24086), .decimal = LSC_F2I_F(1.24086)}, {.integer = LSC_F2I_I(1.33480), .decimal = LSC_F2I_F(1.33480)}, {.integer = LSC_F2I_I(1.44811), .decimal = LSC_F2I_F(1.44811)}, {.integer = LSC_F2I_I(1.57655), .decimal = LSC_F2I_F(1.57655)}, {.integer = LSC_F2I_I(1.71775), .decimal = LSC_F2I_F(1.71775)}, {.integer = LSC_F2I_I(1.71775), .decimal = LSC_F2I_F(1.71775)},
    {.integer = LSC_F2I_I(1.76762), .decimal = LSC_F2I_F(1.76762)}, {.integer = LSC_F2I_I(1.60024), .decimal = LSC_F2I_F(1.60024)}, {.integer = LSC_F2I_I(1.45672), .decimal = LSC_F2I_F(1.45672)}, {.integer = LSC_F2I_I(1.33666), .decimal = LSC_F2I_F(1.33666)}, {.integer = LSC_F2I_I(1.23399), .decimal = LSC_F2I_F(1.23399)}, {.integer = LSC_F2I_I(1.15346), .decimal = LSC_F2I_F(1.15346)}, {.integer = LSC_F2I_I(1.09332), .decimal = LSC_F2I_F(1.09332)}, {.integer = LSC_F2I_I(1.05147), .decimal = LSC_F2I_F(1.05147)}, {.integer = LSC_F2I_I(1.02427), .decimal = LSC_F2I_F(1.02427)}, {.integer = LSC_F2I_I(1.01183), .decimal = LSC_F2I_F(1.01183)}, {.integer = LSC_F2I_I(1.01212), .decimal = LSC_F2I_F(1.01212)}, {.integer = LSC_F2I_I(1.02083), .decimal = LSC_F2I_F(1.02083)}, {.integer = LSC_F2I_I(1.04190), .decimal = LSC_F2I_F(1.04190)}, {.integer = LSC_F2I_I(1.08239), .decimal = LSC_F2I_F(1.08239)}, {.integer = LSC_F2I_I(1.13756), .decimal = LSC_F2I_F(1.13756)}, {.integer = LSC_F2I_I(1.21398), .decimal = LSC_F2I_F(1.21398)}, {.integer = LSC_F2I_I(1.30853), .decimal = LSC_F2I_F(1.30853)}, {.integer = LSC_F2I_I(1.41903), .decimal = LSC_F2I_F(1.41903)}, {.integer = LSC_F2I_I(1.54563), .decimal = LSC_F2I_F(1.54563)}, {.integer = LSC_F2I_I(1.68974), .decimal = LSC_F2I_F(1.68974)}, {.integer = LSC_F2I_I(1.68974), .decimal = LSC_F2I_F(1.68974)},
    {.integer = LSC_F2I_I(1.76084), .decimal = LSC_F2I_F(1.76084)}, {.integer = LSC_F2I_I(1.59295), .decimal = LSC_F2I_F(1.59295)}, {.integer = LSC_F2I_I(1.44517), .decimal = LSC_F2I_F(1.44517)}, {.integer = LSC_F2I_I(1.32544), .decimal = LSC_F2I_F(1.32544)}, {.integer = LSC_F2I_I(1.22396), .decimal = LSC_F2I_F(1.22396)}, {.integer = LSC_F2I_I(1.14357), .decimal = LSC_F2I_F(1.14357)}, {.integer = LSC_F2I_I(1.08062), .decimal = LSC_F2I_F(1.08062)}, {.integer = LSC_F2I_I(1.03833), .decimal = LSC_F2I_F(1.03833)}, {.integer = LSC_F2I_I(1.01054), .decimal = LSC_F2I_F(1.01054)}, {.integer = LSC_F2I_I(0.99947), .decimal = LSC_F2I_F(0.99947)}, {.integer = LSC_F2I_I(0.99681), .decimal = LSC_F2I_F(0.99681)}, {.integer = LSC_F2I_I(1.00721), .decimal = LSC_F2I_F(1.00721)}, {.integer = LSC_F2I_I(1.03078), .decimal = LSC_F2I_F(1.03078)}, {.integer = LSC_F2I_I(1.06957), .decimal = LSC_F2I_F(1.06957)}, {.integer = LSC_F2I_I(1.12527), .decimal = LSC_F2I_F(1.12527)}, {.integer = LSC_F2I_I(1.20079), .decimal = LSC_F2I_F(1.20079)}, {.integer = LSC_F2I_I(1.29592), .decimal = LSC_F2I_F(1.29592)}, {.integer = LSC_F2I_I(1.40549), .decimal = LSC_F2I_F(1.40549)}, {.integer = LSC_F2I_I(1.53594), .decimal = LSC_F2I_F(1.53594)}, {.integer = LSC_F2I_I(1.67702), .decimal = LSC_F2I_F(1.67702)}, {.integer = LSC_F2I_I(1.67702), .decimal = LSC_F2I_F(1.67702)},
    {.integer = LSC_F2I_I(1.76656), .decimal = LSC_F2I_F(1.76656)}, {.integer = LSC_F2I_I(1.59807), .decimal = LSC_F2I_F(1.59807)}, {.integer = LSC_F2I_I(1.45543), .decimal = LSC_F2I_F(1.45543)}, {.integer = LSC_F2I_I(1.33195), .decimal = LSC_F2I_F(1.33195)}, {.integer = LSC_F2I_I(1.22880), .decimal = LSC_F2I_F(1.22880)}, {.integer = LSC_F2I_I(1.14914), .decimal = LSC_F2I_F(1.14914)}, {.integer = LSC_F2I_I(1.08621), .decimal = LSC_F2I_F(1.08621)}, {.integer = LSC_F2I_I(1.04372), .decimal = LSC_F2I_F(1.04372)}, {.integer = LSC_F2I_I(1.01379), .decimal = LSC_F2I_F(1.01379)}, {.integer = LSC_F2I_I(0.99941), .decimal = LSC_F2I_F(0.99941)}, {.integer = LSC_F2I_I(1.00000), .decimal = LSC_F2I_F(1.00000)}, {.integer = LSC_F2I_I(1.00993), .decimal = LSC_F2I_F(1.00993)}, {.integer = LSC_F2I_I(1.03346), .decimal = LSC_F2I_F(1.03346)}, {.integer = LSC_F2I_I(1.07557), .decimal = LSC_F2I_F(1.07557)}, {.integer = LSC_F2I_I(1.13147), .decimal = LSC_F2I_F(1.13147)}, {.integer = LSC_F2I_I(1.20665), .decimal = LSC_F2I_F(1.20665)}, {.integer = LSC_F2I_I(1.30215), .decimal = LSC_F2I_F(1.30215)}, {.integer = LSC_F2I_I(1.41653), .decimal = LSC_F2I_F(1.41653)}, {.integer = LSC_F2I_I(1.54397), .decimal = LSC_F2I_F(1.54397)}, {.integer = LSC_F2I_I(1.68580), .decimal = LSC_F2I_F(1.68580)}, {.integer = LSC_F2I_I(1.68580), .decimal = LSC_F2I_F(1.68580)},
    {.integer = LSC_F2I_I(1.79871), .decimal = LSC_F2I_F(1.79871)}, {.integer = LSC_F2I_I(1.63004), .decimal = LSC_F2I_F(1.63004)}, {.integer = LSC_F2I_I(1.48396), .decimal = LSC_F2I_F(1.48396)}, {.integer = LSC_F2I_I(1.35812), .decimal = LSC_F2I_F(1.35812)}, {.integer = LSC_F2I_I(1.25525), .decimal = LSC_F2I_F(1.25525)}, {.integer = LSC_F2I_I(1.17180), .decimal = LSC_F2I_F(1.17180)}, {.integer = LSC_F2I_I(1.10714), .decimal = LSC_F2I_F(1.10714)}, {.integer = LSC_F2I_I(1.06158), .decimal = LSC_F2I_F(1.06158)}, {.integer = LSC_F2I_I(1.03347), .decimal = LSC_F2I_F(1.03347)}, {.integer = LSC_F2I_I(1.01938), .decimal = LSC_F2I_F(1.01938)}, {.integer = LSC_F2I_I(1.01687), .decimal = LSC_F2I_F(1.01687)}, {.integer = LSC_F2I_I(1.03014), .decimal = LSC_F2I_F(1.03014)}, {.integer = LSC_F2I_I(1.05452), .decimal = LSC_F2I_F(1.05452)}, {.integer = LSC_F2I_I(1.09545), .decimal = LSC_F2I_F(1.09545)}, {.integer = LSC_F2I_I(1.15594), .decimal = LSC_F2I_F(1.15594)}, {.integer = LSC_F2I_I(1.23373), .decimal = LSC_F2I_F(1.23373)}, {.integer = LSC_F2I_I(1.33027), .decimal = LSC_F2I_F(1.33027)}, {.integer = LSC_F2I_I(1.44016), .decimal = LSC_F2I_F(1.44016)}, {.integer = LSC_F2I_I(1.57002), .decimal = LSC_F2I_F(1.57002)}, {.integer = LSC_F2I_I(1.71360), .decimal = LSC_F2I_F(1.71360)}, {.integer = LSC_F2I_I(1.71360), .decimal = LSC_F2I_F(1.71360)},
    {.integer = LSC_F2I_I(1.85020), .decimal = LSC_F2I_F(1.85020)}, {.integer = LSC_F2I_I(1.67330), .decimal = LSC_F2I_F(1.67330)}, {.integer = LSC_F2I_I(1.52843), .decimal = LSC_F2I_F(1.52843)}, {.integer = LSC_F2I_I(1.40479), .decimal = LSC_F2I_F(1.40479)}, {.integer = LSC_F2I_I(1.29543), .decimal = LSC_F2I_F(1.29543)}, {.integer = LSC_F2I_I(1.21541), .decimal = LSC_F2I_F(1.21541)}, {.integer = LSC_F2I_I(1.14812), .decimal = LSC_F2I_F(1.14812)}, {.integer = LSC_F2I_I(1.10131), .decimal = LSC_F2I_F(1.10131)}, {.integer = LSC_F2I_I(1.07022), .decimal = LSC_F2I_F(1.07022)}, {.integer = LSC_F2I_I(1.05528), .decimal = LSC_F2I_F(1.05528)}, {.integer = LSC_F2I_I(1.05302), .decimal = LSC_F2I_F(1.05302)}, {.integer = LSC_F2I_I(1.06670), .decimal = LSC_F2I_F(1.06670)}, {.integer = LSC_F2I_I(1.09176), .decimal = LSC_F2I_F(1.09176)}, {.integer = LSC_F2I_I(1.13876), .decimal = LSC_F2I_F(1.13876)}, {.integer = LSC_F2I_I(1.19971), .decimal = LSC_F2I_F(1.19971)}, {.integer = LSC_F2I_I(1.27945), .decimal = LSC_F2I_F(1.27945)}, {.integer = LSC_F2I_I(1.37484), .decimal = LSC_F2I_F(1.37484)}, {.integer = LSC_F2I_I(1.48622), .decimal = LSC_F2I_F(1.48622)}, {.integer = LSC_F2I_I(1.61818), .decimal = LSC_F2I_F(1.61818)}, {.integer = LSC_F2I_I(1.75785), .decimal = LSC_F2I_F(1.75785)}, {.integer = LSC_F2I_I(1.75785), .decimal = LSC_F2I_F(1.75785)},
    {.integer = LSC_F2I_I(1.93067), .decimal = LSC_F2I_F(1.93067)}, {.integer = LSC_F2I_I(1.75130), .decimal = LSC_F2I_F(1.75130)}, {.integer = LSC_F2I_I(1.59417), .decimal = LSC_F2I_F(1.59417)}, {.integer = LSC_F2I_I(1.46699), .decimal = LSC_F2I_F(1.46699)}, {.integer = LSC_F2I_I(1.36333), .decimal = LSC_F2I_F(1.36333)}, {.integer = LSC_F2I_I(1.27470), .decimal = LSC_F2I_F(1.27470)}, {.integer = LSC_F2I_I(1.20909), .decimal = LSC_F2I_F(1.20909)}, {.integer = LSC_F2I_I(1.15862), .decimal = LSC_F2I_F(1.15862)}, {.integer = LSC_F2I_I(1.12717), .decimal = LSC_F2I_F(1.12717)}, {.integer = LSC_F2I_I(1.11149), .decimal = LSC_F2I_F(1.11149)}, {.integer = LSC_F2I_I(1.10983), .decimal = LSC_F2I_F(1.10983)}, {.integer = LSC_F2I_I(1.12322), .decimal = LSC_F2I_F(1.12322)}, {.integer = LSC_F2I_I(1.15431), .decimal = LSC_F2I_F(1.15431)}, {.integer = LSC_F2I_I(1.19969), .decimal = LSC_F2I_F(1.19969)}, {.integer = LSC_F2I_I(1.26245), .decimal = LSC_F2I_F(1.26245)}, {.integer = LSC_F2I_I(1.34375), .decimal = LSC_F2I_F(1.34375)}, {.integer = LSC_F2I_I(1.43890), .decimal = LSC_F2I_F(1.43890)}, {.integer = LSC_F2I_I(1.55024), .decimal = LSC_F2I_F(1.55024)}, {.integer = LSC_F2I_I(1.67795), .decimal = LSC_F2I_F(1.67795)}, {.integer = LSC_F2I_I(1.82316), .decimal = LSC_F2I_F(1.82316)}, {.integer = LSC_F2I_I(1.82316), .decimal = LSC_F2I_F(1.82316)},
    {.integer = LSC_F2I_I(2.02113), .decimal = LSC_F2I_F(2.02113)}, {.integer = LSC_F2I_I(1.84756), .decimal = LSC_F2I_F(1.84756)}, {.integer = LSC_F2I_I(1.69007), .decimal = LSC_F2I_F(1.69007)}, {.integer = LSC_F2I_I(1.55318), .decimal = LSC_F2I_F(1.55318)}, {.integer = LSC_F2I_I(1.44582), .decimal = LSC_F2I_F(1.44582)}, {.integer = LSC_F2I_I(1.35755), .decimal = LSC_F2I_F(1.35755)}, {.integer = LSC_F2I_I(1.28553), .decimal = LSC_F2I_F(1.28553)}, {.integer = LSC_F2I_I(1.23769), .decimal = LSC_F2I_F(1.23769)}, {.integer = LSC_F2I_I(1.20629), .decimal = LSC_F2I_F(1.20629)}, {.integer = LSC_F2I_I(1.19030), .decimal = LSC_F2I_F(1.19030)}, {.integer = LSC_F2I_I(1.18856), .decimal = LSC_F2I_F(1.18856)}, {.integer = LSC_F2I_I(1.20463), .decimal = LSC_F2I_F(1.20463)}, {.integer = LSC_F2I_I(1.23439), .decimal = LSC_F2I_F(1.23439)}, {.integer = LSC_F2I_I(1.28290), .decimal = LSC_F2I_F(1.28290)}, {.integer = LSC_F2I_I(1.34676), .decimal = LSC_F2I_F(1.34676)}, {.integer = LSC_F2I_I(1.42415), .decimal = LSC_F2I_F(1.42415)}, {.integer = LSC_F2I_I(1.52148), .decimal = LSC_F2I_F(1.52148)}, {.integer = LSC_F2I_I(1.63545), .decimal = LSC_F2I_F(1.63545)}, {.integer = LSC_F2I_I(1.76114), .decimal = LSC_F2I_F(1.76114)}, {.integer = LSC_F2I_I(1.91111), .decimal = LSC_F2I_F(1.91111)}, {.integer = LSC_F2I_I(1.91111), .decimal = LSC_F2I_F(1.91111)},
    {.integer = LSC_F2I_I(2.09217), .decimal = LSC_F2I_F(2.09217)}, {.integer = LSC_F2I_I(1.91941), .decimal = LSC_F2I_F(1.91941)}, {.integer = LSC_F2I_I(1.76635), .decimal = LSC_F2I_F(1.76635)}, {.integer = LSC_F2I_I(1.62092), .decimal = LSC_F2I_F(1.62092)}, {.integer = LSC_F2I_I(1.51100), .decimal = LSC_F2I_F(1.51100)}, {.integer = LSC_F2I_I(1.42151), .decimal = LSC_F2I_F(1.42151)}, {.integer = LSC_F2I_I(1.35206), .decimal = LSC_F2I_F(1.35206)}, {.integer = LSC_F2I_I(1.29685), .decimal = LSC_F2I_F(1.29685)}, {.integer = LSC_F2I_I(1.26468), .decimal = LSC_F2I_F(1.26468)}, {.integer = LSC_F2I_I(1.25030), .decimal = LSC_F2I_F(1.25030)}, {.integer = LSC_F2I_I(1.24831), .decimal = LSC_F2I_F(1.24831)}, {.integer = LSC_F2I_I(1.26332), .decimal = LSC_F2I_F(1.26332)}, {.integer = LSC_F2I_I(1.29867), .decimal = LSC_F2I_F(1.29867)}, {.integer = LSC_F2I_I(1.34189), .decimal = LSC_F2I_F(1.34189)}, {.integer = LSC_F2I_I(1.40850), .decimal = LSC_F2I_F(1.40850)}, {.integer = LSC_F2I_I(1.48753), .decimal = LSC_F2I_F(1.48753)}, {.integer = LSC_F2I_I(1.58368), .decimal = LSC_F2I_F(1.58368)}, {.integer = LSC_F2I_I(1.69511), .decimal = LSC_F2I_F(1.69511)}, {.integer = LSC_F2I_I(1.82586), .decimal = LSC_F2I_F(1.82586)}, {.integer = LSC_F2I_I(1.96868), .decimal = LSC_F2I_F(1.96868)}, {.integer = LSC_F2I_I(1.96868), .decimal = LSC_F2I_F(1.96868)},
    {.integer = LSC_F2I_I(2.09217), .decimal = LSC_F2I_F(2.09217)}, {.integer = LSC_F2I_I(1.91941), .decimal = LSC_F2I_F(1.91941)}, {.integer = LSC_F2I_I(1.76635), .decimal = LSC_F2I_F(1.76635)}, {.integer = LSC_F2I_I(1.62092), .decimal = LSC_F2I_F(1.62092)}, {.integer = LSC_F2I_I(1.51100), .decimal = LSC_F2I_F(1.51100)}, {.integer = LSC_F2I_I(1.42151), .decimal = LSC_F2I_F(1.42151)}, {.integer = LSC_F2I_I(1.35206), .decimal = LSC_F2I_F(1.35206)}, {.integer = LSC_F2I_I(1.29685), .decimal = LSC_F2I_F(1.29685)}, {.integer = LSC_F2I_I(1.26468), .decimal = LSC_F2I_F(1.26468)}, {.integer = LSC_F2I_I(1.25030), .decimal = LSC_F2I_F(1.25030)}, {.integer = LSC_F2I_I(1.24831), .decimal = LSC_F2I_F(1.24831)}, {.integer = LSC_F2I_I(1.26332), .decimal = LSC_F2I_F(1.26332)}, {.integer = LSC_F2I_I(1.29867), .decimal = LSC_F2I_F(1.29867)}, {.integer = LSC_F2I_I(1.34189), .decimal = LSC_F2I_F(1.34189)}, {.integer = LSC_F2I_I(1.40850), .decimal = LSC_F2I_F(1.40850)}, {.integer = LSC_F2I_I(1.48753), .decimal = LSC_F2I_F(1.48753)}, {.integer = LSC_F2I_I(1.58368), .decimal = LSC_F2I_F(1.58368)}, {.integer = LSC_F2I_I(1.69511), .decimal = LSC_F2I_F(1.69511)}, {.integer = LSC_F2I_I(1.82586), .decimal = LSC_F2I_F(1.82586)}, {.integer = LSC_F2I_I(1.96868), .decimal = LSC_F2I_F(1.96868)}, {.integer = LSC_F2I_I(1.96868), .decimal = LSC_F2I_F(1.96868)}
};

#endif

#if EXP_GAIN_PARA_EN
// exposure in us
static void config_exposure_time(int cam_fd, int32_t exposure)
{
    struct v4l2_query_ext_ctrl qctrl;
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];

    qctrl.id = V4L2_CID_EXPOSURE;
    ioctl(cam_fd, VIDIOC_QUERY_EXT_CTRL, &qctrl);
    ESP_LOGW(TAG, "EXP min: %d, EXP max:  %d, step:%d", (int)qctrl.minimum, (int)qctrl.maximum, (int)qctrl.step);

    controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_EXPOSURE;
    control[0].value    = exposure;
    if (ioctl(cam_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGE(TAG, "failed to set exposure time");
    } else {
        ESP_LOGW(TAG, "set exposure");
    }
}
// total gain in float
static void config_pixel_gain(int cam_fd, float config_gain)
{
    esp_err_t ret;
    int fd = cam_fd;
    struct v4l2_querymenu qmenu;
    struct v4l2_query_ext_ctrl qctrl;
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];

    int32_t gain_value = 0;
    int32_t index = -1;

    qctrl.id = V4L2_CID_GAIN;
    ret = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qctrl);
    if (ret) {
        ESP_LOGE(TAG, "failed to query gain");
        return;
    } else {
        ESP_LOGW(TAG, "Gain min: %d, Gain max:  %d", (int)qctrl.minimum, (int)qctrl.maximum);
    }

    for (int32_t i = qctrl.minimum; i < qctrl.maximum; i++) {
        int32_t gain0;
        int32_t gain1;

        qmenu.id = V4L2_CID_GAIN;
        qmenu.index = i;
        ret = ioctl(fd, VIDIOC_QUERYMENU, &qmenu);
        if (ret) {
            ESP_LOGE(TAG, "failed to query gain min menu");
            return;
        }
        gain0 = qmenu.value;

        if (i == qctrl.minimum) {
            gain_value = gain0 * config_gain;
        }

        qmenu.id = V4L2_CID_GAIN;
        qmenu.index = i + 1;
        ret = ioctl(fd, VIDIOC_QUERYMENU, &qmenu);
        if (ret) {
            ESP_LOGE(TAG, "failed to query gain min menu");
            return;
        }
        gain1 = qmenu.value;

        if ((gain_value >= gain0) && (gain_value <= gain1)) {
            uint32_t len_1st = gain_value - gain0;
            uint32_t len_2nd = gain1 - gain_value;

            ESP_LOGD(TAG, "[%" PRIu32 ", %" PRIu32 "]", gain0, gain1);

            if (len_1st > len_2nd) {
                index = i + 1;
            } else {
                index = i;
            }
        }
    }

    if (index >= 0) {
        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_GAIN;
        control[0].value    = index;
        if (ioctl(cam_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set pixel gain");
        } else {
            ESP_LOGW(TAG, "set GAIN");
        }
    } else {
        // ESP_LOGE(TAG, "failed to find %0.4f", config_gain);
    }
}
#endif

#if WB_PARA_EN
static void config_white_balance(int isp_fd, float r_g, float b_g)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];

    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_RED_BALANCE;
    control[0].value    = r_g * V4L2_CID_RED_BALANCE_DEN;
    if (ioctl(isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGE(TAG, "failed to set red balance");
    }

    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_BLUE_BALANCE;
    control[0].value    = b_g * V4L2_CID_BLUE_BALANCE_DEN;
    if (ioctl(isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGE(TAG, "failed to set blue balance");
    }
}
#endif

esp_err_t init_isp_dev(int cam_fd)
{
    int fd;
    esp_err_t ret;
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];

    fd = open(ESP_VIDEO_ISP1_DEVICE_NAME, O_RDWR);
    ESP_RETURN_ON_FALSE(fd > 0, ESP_ERR_INVALID_ARG, TAG, "failed to open %s", "isp");

#if EXP_GAIN_PARA_EN
    config_exposure_time(cam_fd, 30 * 1000); // 30ms
    config_pixel_gain(cam_fd, 3.000); // toatl gain
#endif

#if BLC_PARA_EN
    // esp_video_isp_blc_t blc = {
    //     .enable = true,
    //     .top_left_offset = 16,
    //     .top_right_offset = 16,
    //     .bottom_left_offset = 16,
    //     .bottom_right_offset = 16,
    // };
    esp_video_isp_blc_t blc = {
        .enable = true,
        .top_left_offset = 0x10,
        .top_right_offset = 0x10,
        .bottom_left_offset = 0x10,
        .bottom_right_offset = 0x10,
    };

    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_USER_ESP_ISP_BLC;
    control[0].p_u8     = (uint8_t *)&blc;
    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to initialize blc");
#endif

#if BF_PARA_EN
    esp_video_isp_bf_t bf = {
        .enable = true,
        .level = 3, //gain large matrix large
        .matrix = {
            {1, 1, 1},
            {1, 2, 1},
            {1, 1, 1},
        }
    };
    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_USER_ESP_ISP_BF;
    control[0].p_u8     = (uint8_t *)&bf;
    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to initialize bayer filter");
#endif

#if LSC_PARA_EN
    esp_video_isp_lsc_t lsc_cfg = {
        .enable = true,
        .gain_r = r_gain,
        .gain_gr = gr_gain,
        .gain_gb = gb_gain,
        .gain_b = b_gain,
        .lsc_gain_size = lsc_gain_size,
    };

    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_USER_ESP_ISP_LSC;
    control[0].p_u8     = (uint8_t *)&lsc_cfg;
    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to initialize lsc");
    printf("r[0].i=%x, r[0].d=%x\r\n", r_gain[0].integer, r_gain[0].decimal);
#endif

#if DEM_PARA_EN
    esp_video_isp_demosaic_t dem = {
        .enable = true,
        .gradient_ratio = 1.0,
    };
    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_USER_ESP_ISP_DEMOSAIC;
    control[0].p_u8     = (uint8_t *)&dem;
    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to initialize dem");
#endif

#if SHARPEN_PARA_EN
    esp_video_isp_sharpen_t sharpen = {
        .enable = true,
        .h_thresh = 40,
        .l_thresh = 6,
        .h_coeff = 1.225,
        .m_coeff = 1.45,
        .matrix = {
            {1, 1, 1},
            {1, 1, 1},
            {1, 1, 1}
        }
    };

    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_USER_ESP_ISP_SHARPEN;
    control[0].p_u8     = (uint8_t *)&sharpen;
    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to initialize sharpen");
#endif

#if GAMMA_PARA_EN
    double gamma_pram = 0.6;
    double gain = 1.0 / pow((double)(UINT8_MAX + 1 - (UINT8_MAX / ISP_GAMMA_CURVE_POINTS_NUM)) / UINT8_MAX, gamma_pram);
    esp_video_isp_gamma_t gamma = {
        .enable = true
    };

    for (int i = 0; i < ISP_GAMMA_CURVE_POINTS_NUM - 1; i++) {
        gamma.points[i].x = MIN((i + 1) * (256 / ISP_GAMMA_CURVE_POINTS_NUM), 255);
        gamma.points[i].y = MIN((uint8_t)(pow((double)gamma.points[i].x / UINT8_MAX, gamma_pram) * UINT8_MAX * gain + 0.5), 255);
    }
    gamma.points[ISP_GAMMA_CURVE_POINTS_NUM - 1].x = 255;
    gamma.points[ISP_GAMMA_CURVE_POINTS_NUM - 1].y = 255;

    // gamma.points[0].y = 18;
    // gamma.points[1].y = 38;
    // gamma.points[2].y = 56;
    for (int i = 0; i < ISP_GAMMA_CURVE_POINTS_NUM; i++) {
        ESP_LOGW(TAG, "%d: x=%d, y=%d", i, gamma.points[i].x, gamma.points[i].y);
    }

    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_USER_ESP_ISP_GAMMA;
    control[0].p_u8     = (uint8_t *)&gamma;
    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to initialize GAMMA");
#endif

#if WB_PARA_EN
    config_white_balance(fd, 2.0381, 1.7009); // V4L2_CID_USER_ESP_ISP_WB //255==200
#endif

#if CCM_PARA_EN
    esp_video_isp_ccm_t ccm = {
        .enable = true,
        .matrix = {
            // {1.0,  0.0, 0.0},
            // {0.0,  1.0, 0.0},
            // {0.0,  0.0, 1.0},

            {2.0263, -0.7904, -0.2358},
            {-0.3715, 1.5908, -0.2193},
            {-0.3328, -0.4288, 1.7615},
        }
    };

    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_USER_ESP_ISP_CCM;
    control[0].p_u8     = (uint8_t *)&ccm;
    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to initialize CCM");
#endif

#if COLOR_PARA_EN
#if 1
    uint32_t contrast_val = 0x86;

    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_CONTRAST;
    control[0].value     = contrast_val;
    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to initialize constart");
#endif

#if 1
    uint32_t sat_val = 0x80;
    // uint32_t sat_val = 0x0;

    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_SATURATION;
    control[0].value     = sat_val;
    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to initialize sat");
#endif

#if 1
    uint32_t bright_val = 0x00;
    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_BRIGHTNESS;
    control[0].value     = bright_val;
    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to initialize brightness");
#endif

#if 1
    uint32_t hue_val = 0x00;
    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_HUE;
    control[0].value     = hue_val;
    ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to initialize hue");
#endif

#endif

    return ESP_OK;

fail_0:
    close(fd);
    return ret;
}
