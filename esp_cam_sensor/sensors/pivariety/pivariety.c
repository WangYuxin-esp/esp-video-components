/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
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
#include "pivariety_settings.h"
#include "v4l2_cid.h"
#include "pivariety.h"

/*
 * PIVARIETY camera sensor gain control.
 * Note1: The analog gain only has coarse gain, and no fine gain, so in the adjustment of analog gain.
 * Digital gain needs to replace analog fine gain for smooth transition, so as to avoid AGC oscillation.
 * Note2: the analog gain of PIVARIETY will be affected by temperature, it is recommended to increase Dgain first and then Again.
 */
typedef struct {
    uint8_t dgain_fine; // digital gain fine
    uint8_t dgain_coarse; // digital gain coarse
    uint8_t analog_gain;
} pivariety_gain_t;

typedef struct {
    uint32_t exposure_val;
    uint32_t exposure_max;
    uint32_t gain_index; // current gain index

    uint32_t vflip_en : 1;
    uint32_t hmirror_en : 1;
} pivariety_para_t;

struct pivariety_cam {
    pivariety_para_t pivariety_para;
};

#define PIVARIETY_IO_MUX_LOCK(mux)
#define PIVARIETY_IO_MUX_UNLOCK(mux)
#define PIVARIETY_ENABLE_OUT_XCLK(pin,clk)
#define PIVARIETY_DISABLE_OUT_XCLK(pin)

#define PIVARIETY_EXP_MAX_OFFSET   0x15

#define PIVARIETY_FETCH_EXP_H(val)     (((val) >> 12) & 0xF)
#define PIVARIETY_FETCH_EXP_M(val)     (((val) >> 4) & 0xFF)
#define PIVARIETY_FETCH_EXP_L(val)     (((val) & 0xF) << 4)

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms)  vTaskDelay((ms > portTICK_PERIOD_MS ? ms/ portTICK_PERIOD_MS : 1))
#define PIVARIETY_SUPPORT_NUM CONFIG_CAMERA_PIVARIETY_MAX_SUPPORT

static const uint32_t s_limited_abs_gain = CONFIG_CAMERA_PIVARIETY_ABSOLUTE_GAIN_LIMIT;
static size_t s_limited_abs_gain_index;
static const uint8_t s_pivariety_exp_min = 0x09;
static const char *TAG = "pivariety";
#define PIVARIETY_EXPOSURE_TEST_EN 0
#define PIVARIETY_EXPOSURE_TEST_EN_GAIN 0

#define EXPOSURE_V4L2_UNIT_US                   100
#define EXPOSURE_V4L2_TO_PIVARIETY(v, sf)          \
    ((uint32_t)(((double)v) * EXPOSURE_V4L2_UNIT_US * 1000 / (((sf)->isp_info->isp_v1_info.tline_ns)) + 0.5))
#define EXPOSURE_PIVARIETY_TO_V4L2(v, sf)          \
    ((int32_t)(((double)v) * (((sf)->isp_info->isp_v1_info.tline_ns)) / EXPOSURE_V4L2_UNIT_US / 1000 + 0.5))

//  gain = analog_gain  x 1000(To avoid decimal points, the final abs_gain is multiplied by 1000. total gain = 22.26 times)
static const uint32_t pivariety_abs_gain_val_map[] = {
    1000,
    1001,
    1002,
    1003,
    1004,
    1005,
    1006,
    1007,
    1008,
    1009,
    1010,
    1011,
    1012,
    1013,
    1014,
    1015,
    1016,
    1017,
    1018,
    1019,
    1020,
    1021,
    1022,
    1023,
    1024,
    1025,
    1026,
    1027,
    1028,
    1029,
    1030,
    1031,
    1032,
    1033,
    1034,
    1035,
    1036,
    1037,
    1039,
    1040,
    1041,
    1042,
    1043,
    1044,
    1045,
    1046,
    1047,
    1048,
    1049,
    1050,
    1051,
    1052,
    1053,
    1055,
    1056,
    1057,
    1058,
    1059,
    1060,
    1061,
    1062,
    1063,
    1064,
    1066,
    1067,
    1068,
    1069,
    1070,
    1071,
    1072,
    1073,
    1075,
    1076,
    1077,
    1078,
    1079,
    1080,
    1081,
    1082,
    1084,
    1085,
    1086,
    1087,
    1088,
    1089,
    1091,
    1092,
    1093,
    1094,
    1095,
    1096,
    1098,
    1099,
    1100,
    1101,
    1102,
    1103,
    1105,
    1106,
    1107,
    1108,
    1109,
    1111,
    1112,
    1113,
    1114,
    1115,
    1117,
    1118,
    1119,
    1120,
    1122,
    1123,
    1124,
    1125,
    1127,
    1128,
    1129,
    1130,
    1131,
    1133,
    1134,
    1135,
    1137,
    1138,
    1139,
    1140,
    1142,
    1143,
    1144,
    1145,
    1147,
    1148,
    1149,
    1151,
    1152,
    1153,
    1154,
    1156,
    1157,
    1158,
    1160,
    1161,
    1162,
    1164,
    1165,
    1166,
    1168,
    1169,
    1170,
    1172,
    1173,
    1174,
    1176,
    1177,
    1178,
    1180,
    1181,
    1182,
    1184,
    1185,
    1187,
    1188,
    1189,
    1191,
    1192,
    1193,
    1195,
    1196,
    1198,
    1199,
    1200,
    1202,
    1203,
    1205,
    1206,
    1208,
    1209,
    1210,
    1212,
    1213,
    1215,
    1216,
    1218,
    1219,
    1221,
    1222,
    1223,
    1225,
    1226,
    1228,
    1229,
    1231,
    1232,
    1234,
    1235,
    1237,
    1238,
    1240,
    1241,
    1243,
    1244,
    1246,
    1247,
    1249,
    1250,
    1252,
    1253,
    1255,
    1256,
    1258,
    1260,
    1261,
    1263,
    1264,
    1266,
    1267,
    1269,
    1270,
    1272,
    1274,
    1275,
    1277,
    1278,
    1280,
    1282,
    1283,
    1285,
    1286,
    1288,
    1290,
    1291,
    1293,
    1295,
    1296,
    1298,
    1299,
    1301,
    1303,
    1304,
    1306,
    1308,
    1309,
    1311,
    1313,
    1315,
    1316,
    1318,
    1320,
    1321,
    1323,
    1325,
    1326,
    1328,
    1330,
    1332,
    1333,
    1335,
    1337,
    1339,
    1340,
    1342,
    1344,
    1346,
    1347,
    1349,
    1351,
    1353,
    1354,
    1356,
    1358,
    1360,
    1362,
    1364,
    1365,
    1367,
    1369,
    1371,
    1373,
    1374,
    1376,
    1378,
    1380,
    1382,
    1384,
    1386,
    1388,
    1389,
    1391,
    1393,
    1395,
    1397,
    1399,
    1401,
    1403,
    1405,
    1407,
    1409,
    1410,
    1412,
    1414,
    1416,
    1418,
    1420,
    1422,
    1424,
    1426,
    1428,
    1430,
    1432,
    1434,
    1436,
    1438,
    1440,
    1442,
    1444,
    1446,
    1448,
    1450,
    1452,
    1455,
    1457,
    1459,
    1461,
    1463,
    1465,
    1467,
    1469,
    1471,
    1473,
    1476,
    1478,
    1480,
    1482,
    1484,
    1486,
    1488,
    1491,
    1493,
    1495,
    1497,
    1499,
    1501,
    1504,
    1506,
    1508,
    1510,
    1513,
    1515,
    1517,
    1519,
    1522,
    1524,
    1526,
    1528,
    1531,
    1533,
    1535,
    1538,
    1540,
    1542,
    1544,
    1547,
    1549,
    1552,
    1554,
    1556,
    1559,
    1561,
    1563,
    1566,
    1568,
    1571,
    1573,
    1575,
    1578,
    1580,
    1583,
    1585,
    1588,
    1590,
    1593,
    1595,
    1598,
    1600,
    1603,
    1605,
    1608,
    1610,
    1613,
    1615,
    1618,
    1620,
    1623,
    1625,
    1628,
    1631,
    1633,
    1636,
    1638,
    1641,
    1644,
    1646,
    1649,
    1652,
    1654,
    1657,
    1660,
    1662,
    1665,
    1668,
    1670,
    1673,
    1676,
    1679,
    1681,
    1684,
    1687,
    1690,
    1693,
    1695,
    1698,
    1701,
    1704,
    1707,
    1710,
    1712,
    1715,
    1718,
    1721,
    1724,
    1727,
    1730,
    1733,
    1736,
    1739,
    1741,
    1744,
    1747,
    1750,
    1753,
    1756,
    1759,
    1762,
    1766,
    1769,
    1772,
    1775,
    1778,
    1781,
    1784,
    1787,
    1790,
    1793,
    1796,
    1800,
    1803,
    1806,
    1809,
    1812,
    1816,
    1819,
    1822,
    1825,
    1829,
    1832,
    1835,
    1838,
    1842,
    1845,
    1848,
    1852,
    1855,
    1858,
    1862,
    1865,
    1869,
    1872,
    1875,
    1879,
    1882,
    1886,
    1889,
    1893,
    1896,
    1900,
    1903,
    1907,
    1910,
    1914,
    1918,
    1921,
    1925,
    1928,
    1932,
    1936,
    1939,
    1943,
    1947,
    1950,
    1954,
    1958,
    1962,
    1965,
    1969,
    1973,
    1977,
    1981,
    1984,
    1988,
    1992,
    1996,
    2000,
    2004,
    2008,
    2012,
    2016,
    2020,
    2024,
    2028,
    2032,
    2036,
    2040,
    2044,
    2048,
    2052,
    2056,
    2060,
    2065,
    2069,
    2073,
    2077,
    2081,
    2086,
    2090,
    2094,
    2098,
    2103,
    2107,
    2111,
    2116,
    2120,
    2124,
    2129,
    2133,
    2138,
    2142,
    2147,
    2151,
    2156,
    2160,
    2165,
    2169,
    2174,
    2179,
    2183,
    2188,
    2193,
    2197,
    2202,
    2207,
    2212,
    2216,
    2221,
    2226,
    2231,
    2236,
    2241,
    2246,
    2251,
    2256,
    2260,
    2265,
    2271,
    2276,
    2281,
    2286,
    2291,
    2296,
    2301,
    2306,
    2312,
    2317,
    2322,
    2327,
    2333,
    2338,
    2343,
    2349,
    2354,
    2359,
    2365,
    2370,
    2376,
    2381,
    2387,
    2393,
    2398,
    2404,
    2409,
    2415,
    2421,
    2427,
    2432,
    2438,
    2444,
    2450,
    2456,
    2462,
    2467,
    2473,
    2479,
    2485,
    2491,
    2498,
    2504,
    2510,
    2516,
    2522,
    2528,
    2535,
    2541,
    2547,
    2554,
    2560,
    2566,
    2573,
    2579,
    2586,
    2592,
    2599,
    2606,
    2612,
    2619,
    2626,
    2632,
    2639,
    2646,
    2653,
    2660,
    2667,
    2674,
    2681,
    2688,
    2695,
    2702,
    2709,
    2716,
    2723,
    2731,
    2738,
    2745,
    2753,
    2760,
    2768,
    2775,
    2783,
    2790,
    2798,
    2805,
    2813,
    2821,
    2829,
    2837,
    2844,
    2852,
    2860,
    2868,
    2876,
    2885,
    2893,
    2901,
    2909,
    2917,
    2926,
    2934,
    2943,
    2951,
    2960,
    2968,
    2977,
    2985,
    2994,
    3003,
    3012,
    3021,
    3030,
    3039,
    3048,
    3057,
    3066,
    3075,
    3084,
    3094,
    3103,
    3112,
    3122,
    3131,
    3141,
    3151,
    3160,
    3170,
    3180,
    3190,
    3200,
    3210,
    3220,
    3230,
    3241,
    3251,
    3261,
    3272,
    3282,
    3293,
    3303,
    3314,
    3325,
    3336,
    3346,
    3357,
    3368,
    3380,
    3391,
    3402,
    3413,
    3425,
    3436,
    3448,
    3459,
    3471,
    3483,
    3495,
    3507,
    3519,
    3531,
    3543,
    3556,
    3568,
    3580,
    3593,
    3606,
    3618,
    3631,
    3644,
    3657,
    3670,
    3683,
    3697,
    3710,
    3724,
    3737,
    3751,
    3765,
    3779,
    3793,
    3807,
    3821,
    3835,
    3850,
    3864,
    3879,
    3894,
    3908,
    3923,
    3938,
    3954,
    3969,
    3984,
    4000,
    4016,
    4031,
    4047,
    4063,
    4080,
    4096,
    4112,
    4129,
    4146,
    4163,
    4180,
    4197,
    4214,
    4231,
    4249,
    4267,
    4285,
    4303,
    4321,
    4339,
    4357,
    4376,
    4395,
    4414,
    4433,
    4452,
    4472,
    4491,
    4511,
    4531,
    4551,
    4571,
    4592,
    4613,
    4633,
    4655,
    4676,
    4697,
    4719,
    4741,
    4763,
    4785,
    4808,
    4830,
    4853,
    4876,
    4900,
    4923,
    4947,
    4971,
    4995,
    5020,
    5044,
    5069,
    5095,
    5120,
    5146,
    5172,
    5198,
    5224,
    5251,
    5278,
    5306,
    5333,
    5361,
    5389,
    5418,
    5447,
    5476,
    5505,
    5535,
    5565,
    5596,
    5626,
    5657,
    5689,
    5721,
    5753,
    5785,
    5818,
    5851,
    5885,
    5919,
    5953,
    5988,
    6024,
    6059,
    6095,
    6132,
    6169,
    6206,
    6244,
    6282,
    6321,
    6360,
    6400,
    6440,
    6481,
    6522,
    6564,
    6606,
    6649,
    6693,
    6737,
    6781,
    6827,
    6872,
    6919,
    6966,
    7014,
    7062,
    7111,
    7161,
    7211,
    7262,
    7314,
    7367,
    7420,
    7474,
    7529,
    7585,
    7642,
    7699,
    7758,
    7817,
    7877,
    7938,
    8000,
    8063,
    8127,
    8192,
    8258,
    8325,
    8393,
    8463,
    8533,
    8605,
    8678,
    8752,
    8828,
    8904,
    8982,
    9062,
    9143,
    9225,
    9309,
    9394,
    9481,
    9570,
    9660,
    9752,
    9846,
    9942,
    10039,
    10139,
    10240,
    10343,
    10449,
    10557,
    10667,
    10779,
    10894,
    11011,
    11130,
    11253,
    11378,
    11506,
    11636,
    11770,
    11907,
    12047,
    12190,
    12337,
    12488,
    12642,
    12800,
    12962,
    13128,
    13299,
    13474,
    13653,
    13838,
    14027,
    14222,
    14423,
    14629,
    14841,
    15059,
    15284,
    15515,
    15754,
    16000,
    16254,
    16516,
    16787,
    17067,
    17356,
    17655,
    17965,
    18286,
    18618,
    18963,
    19321,
    19692,
    20078,
    20480,
    20898,
    21333,
    21787,
    22261,
};

static const esp_cam_sensor_isp_info_t pivariety_isp_info[] = {

    {
        .isp_v1_info = {   //1920x1080
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 945000000,
            .vts = 1436,
            .hts = 19167,
            .tline_ns = 20282,
            .gain_def = 500, // gain table index
            .exp_def = 0x2dc,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
    {
        .isp_v1_info = {   //1600x1200
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 945000000,
            .vts = 1602,
            .hts = 16492,
            .tline_ns = 17451,
            .gain_def = 500, // gain table index
            .exp_def = 0x2dc,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
    {
        .isp_v1_info = { //1280x720
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 1900800000,
            .vts = 1602,
            .hts = 17900,
            .tline_ns = 9417,
            .gain_def = 500, // gain table index
            .exp_def = 0x2dc,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
    {
        .isp_v1_info = { //1024x600
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 1900800000,
            .vts = 1802,    //4167,
            .hts = 18900,    //2976,
            .tline_ns = 9617,
            .gain_def = 500, // gain table index
            .exp_def = 0x2dc,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    },
    {
        .isp_v1_info = { //640x480
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 772200000,
            .vts = 3138,    //4167,
            .hts = 8276,    //2976,
            .tline_ns = 9522,
            .gain_def = 500, // gain table index
            .exp_def = 0x2dc,
            .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB,
        }
    }
};

static const esp_cam_sensor_format_t pivariety_format_info[] = {
    {
        .name = "MIPI_2lane_24Minput_RAW10_1920x1080_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1920,
        .height = 1080,
        .regs = pivariety_MIPI_2lane_raw10_1920x1080_30fps,
        .regs_size = ARRAY_SIZE(pivariety_MIPI_2lane_raw10_1920x1080_30fps),
        .fps = 30,
        .isp_info = &pivariety_isp_info[0],
        .mipi_info = {
            .mipi_clk = 480000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_24Minput_RAW10_1600x1200_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1600,
        .height = 1200,
        .regs = pivariety_MIPI_2lane_raw10_1600x1200_30fps,
        .regs_size = ARRAY_SIZE(pivariety_MIPI_2lane_raw10_1600x1200_30fps),
        .fps = 30,
        .isp_info = &pivariety_isp_info[1],
        .mipi_info = {
            .mipi_clk = 480000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_24Minput_RAW10_1280x720_60fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1280,
        .height = 720,
        .regs = pivariety_MIPI_2lane_raw10_1280x720_60fps,
        .regs_size = ARRAY_SIZE(pivariety_MIPI_2lane_raw10_1280x720_60fps),
        .fps = 60,
        .isp_info = &pivariety_isp_info[2],
        .mipi_info = {
            .mipi_clk = 480000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_24Minput_RAW8_1024x600_60fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 1024,
        .height = 600,
        .regs = pivariety_MIPI_2lane_raw10_1024x600_60fps,
        .regs_size = ARRAY_SIZE(pivariety_MIPI_2lane_raw10_1024x600_60fps),
        .fps = 60,
        .isp_info = &pivariety_isp_info[3],
        .mipi_info = {
            .mipi_clk = 640000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_24Minput_RAW8_640x480_120fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 24000000,
        .width = 640,
        .height = 480,
        .regs = pivariety_MIPI_2lane_raw10_640x480_120fps,
        .regs_size = ARRAY_SIZE(pivariety_MIPI_2lane_raw10_640x480_120fps),
        .fps = 30,
        .isp_info = &pivariety_isp_info[4],
        .mipi_info = {
            .mipi_clk = 640000000,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
};

static esp_err_t pivariety_read(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint32_t *read_buf)
{
    return esp_sccb_transmit_receive_reg_a16v32(sccb_handle, reg, read_buf);
}

static esp_err_t pivariety_write(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint32_t data)
{
    return esp_sccb_transmit_reg_a16v32(sccb_handle, reg, data);
}

/* write a array of registers  */
static esp_err_t pivariety_write_array(esp_sccb_io_handle_t sccb_handle, pivariety_reginfo_t *regarray)
{
    int i = 0;
    esp_err_t ret = ESP_OK;
    while ((ret == ESP_OK) && regarray[i].reg != PIVARIETY_REG_END) {
        if (regarray[i].reg != PIVARIETY_REG_DELAY) {
            ret = pivariety_write(sccb_handle, regarray[i].reg, regarray[i].val);
        } else {
            delay_ms(regarray[i].val);
        }
        i++;
    }
    return ret;
}

static esp_err_t pivariety_set_reg_bits(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t offset, uint8_t length, uint8_t value)
{
    return ESP_OK;
}

static esp_err_t pivariety_set_test_pattern(esp_cam_sensor_device_t *dev, int enable)
{
    return ESP_OK;
}

static esp_err_t pivariety_hw_reset(esp_cam_sensor_device_t *dev)
{
    return ESP_OK;
}

static esp_err_t pivariety_soft_reset(esp_cam_sensor_device_t *dev)
{
    return ESP_OK;
}

static esp_err_t pivariety_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t pid;

    ret = pivariety_read(dev->sccb_handle, DEVICE_ID_REG, &pid);
    if (ret != ESP_OK) {
        return ret;
    }
    id->pid = pid ;

    return ret;
}

static esp_err_t pivariety_set_mirror(esp_cam_sensor_device_t *dev, int enable)
{
    return ESP_OK;
}

static esp_err_t pivariety_set_vflip(esp_cam_sensor_device_t *dev, int enable)
{
    return ESP_OK;
}

static esp_err_t pivariety_set_exp_val(esp_cam_sensor_device_t *dev, uint32_t u32_val)
{
    esp_err_t ret;
    struct pivariety_cam *cam_pivariety = (struct pivariety_cam *)dev->priv;
    uint32_t value_buf = MAX(u32_val, s_pivariety_exp_min);
    value_buf = MIN(value_buf, cam_pivariety->pivariety_para.exposure_max);

    ESP_LOGD(TAG, "set exposure 0x%" PRIx32", max:0x%" PRIx32, value_buf, cam_pivariety->pivariety_para.exposure_max);

    ret = pivariety_write(dev->sccb_handle, CTRL_ID_REG, V4L2_CID_EXPOSURE);
    ret |= pivariety_write(dev->sccb_handle, CTRL_VALUE_REG, value_buf);

    if (ret == ESP_OK) {
        cam_pivariety->pivariety_para.exposure_val = value_buf;
    }
    return ret;
}

static esp_err_t pivariety_set_total_gain_val(esp_cam_sensor_device_t *dev, uint32_t u32_val)
{
    esp_err_t ret;
    struct pivariety_cam *cam_pivariety = (struct pivariety_cam *)dev->priv;
    if (u32_val > s_limited_abs_gain_index) {
        u32_val = s_limited_abs_gain_index;
    }

    ESP_LOGD(TAG, "gain %" PRIx32, pivariety_abs_gain_val_map[u32_val]);
    ret = pivariety_write(dev->sccb_handle, CTRL_ID_REG, V4L2_CID_ANALOGUE_GAIN);
    ret |= pivariety_write(dev->sccb_handle, CTRL_VALUE_REG, pivariety_abs_gain_val_map[u32_val]);
    if (ret == ESP_OK) {
        cam_pivariety->pivariety_para.gain_index = u32_val;
    }

    return ret;
}

static esp_err_t pivariety_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    esp_err_t ret = ESP_OK;
    switch (qdesc->id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = s_pivariety_exp_min;
        qdesc->number.maximum = dev->cur_format->isp_info->isp_v1_info.vts - PIVARIETY_EXP_MAX_OFFSET; // max = VTS-6 = height+vblank-6, so when update vblank, exposure_max must be updated
        qdesc->number.step = 1;
        qdesc->default_value = dev->cur_format->isp_info->isp_v1_info.exp_def;
        break;
    case ESP_CAM_SENSOR_EXPOSURE_US:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = MAX(0x01, EXPOSURE_PIVARIETY_TO_V4L2(s_pivariety_exp_min, dev->cur_format)); // The minimum value must be greater than 1
        qdesc->number.maximum = EXPOSURE_PIVARIETY_TO_V4L2((dev->cur_format->isp_info->isp_v1_info.vts - PIVARIETY_EXP_MAX_OFFSET), dev->cur_format);
        qdesc->number.step = MAX(0x01, EXPOSURE_PIVARIETY_TO_V4L2(0x01, dev->cur_format));
        qdesc->default_value = EXPOSURE_PIVARIETY_TO_V4L2((dev->cur_format->isp_info->isp_v1_info.exp_def), dev->cur_format);
        ESP_LOGD(TAG, "exposure: minimum 0x%" PRIx32", maximum 0x%" PRIx32", step 0x%" PRIx32", default_value 0x%" PRIx32, qdesc->number.minimum, qdesc->number.maximum, qdesc->number.step, qdesc->default_value);
        break;
    case ESP_CAM_SENSOR_GAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_ENUMERATION;
        qdesc->enumeration.count = s_limited_abs_gain_index;
        qdesc->enumeration.elements = pivariety_abs_gain_val_map;
        qdesc->default_value = dev->cur_format->isp_info->isp_v1_info.gain_def; // default gain index
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

static esp_err_t pivariety_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;
    struct pivariety_cam *cam_pivariety = (struct pivariety_cam *)dev->priv;
    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL: {
        *(uint32_t *)arg = cam_pivariety->pivariety_para.exposure_val;
        break;
    }
    case ESP_CAM_SENSOR_GAIN: {
        *(uint32_t *)arg = cam_pivariety->pivariety_para.gain_index;
        break;
    }
    default: {
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }
    }
    return ret;
}

static esp_err_t pivariety_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;
    uint32_t u32_val = *(uint32_t *)arg;
    struct pivariety_cam *cam_pivariety = (struct pivariety_cam *)dev->priv;

    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL: {
        uint32_t u32_val = *(uint32_t *)arg;
        ret = pivariety_set_exp_val(dev, u32_val);
        break;
    }
    case ESP_CAM_SENSOR_EXPOSURE_US: {
        uint32_t u32_val = *(uint32_t *)arg;
        uint32_t ori_exp = EXPOSURE_V4L2_TO_PIVARIETY(u32_val, dev->cur_format);
        ret = pivariety_set_exp_val(dev, ori_exp);
        break;
    }
    case ESP_CAM_SENSOR_GAIN: {
        uint32_t u32_val = *(uint32_t *)arg;
        ret = pivariety_set_total_gain_val(dev, u32_val);
        break;
    }
    case ESP_CAM_SENSOR_VFLIP: {
        int *value = (int *)arg;
        ret = pivariety_set_vflip(dev, *value);
        break;
    }
    case ESP_CAM_SENSOR_HMIRROR: {
        int *value = (int *)arg;
        ret = pivariety_set_mirror(dev, *value);
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

static esp_err_t pivariety_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);

    formats->count = ARRAY_SIZE(pivariety_format_info);
    formats->format_array = &pivariety_format_info[0];
    return ESP_OK;
}

static esp_err_t pivariety_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *sensor_cap)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, sensor_cap);

    sensor_cap->fmt_raw = 1;
    return 0;
}

static esp_err_t pivariety_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    struct pivariety_cam *cam_pivariety = (struct pivariety_cam *)dev->priv;
    esp_err_t ret = ESP_OK;
    /* Depending on the interface type, an available configuration is automatically loaded.
    You can set the output format of the sensor without using query_format().*/
    if (format == NULL) {
        format = &pivariety_format_info[CONFIG_CAMERA_PIVARIETY_MIPI_IF_FORMAT_INDEX_DEFAULT];
    }
    delay_ms(100);
    ret = pivariety_write_array(dev->sccb_handle, (pivariety_reginfo_t *)format->regs);
    delay_ms(1000);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set format regs fail");
        return ESP_CAM_SENSOR_ERR_FAILED_SET_FORMAT;
    }

    dev->cur_format = format;
    // init para
    cam_pivariety->pivariety_para.exposure_val = dev->cur_format->isp_info->isp_v1_info.exp_def;
    cam_pivariety->pivariety_para.gain_index = dev->cur_format->isp_info->isp_v1_info.gain_def;
    // set default exp and gain to ensure default info is normal
    pivariety_set_exp_val(dev, dev->cur_format->isp_info->isp_v1_info.exp_def);
    pivariety_set_total_gain_val(dev, dev->cur_format->isp_info->isp_v1_info.gain_def);

    return ret;
}

static esp_err_t pivariety_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
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

#if PIVARIETY_EXPOSURE_TEST_EN
static volatile uint32_t s_exp_v = 0x05;
static bool s_exp_add = true;
TimerHandle_t ae_timer_handle;
static void ae_timer_callback(TimerHandle_t timer)
{
    esp_cam_sensor_device_t *dev = (esp_cam_sensor_device_t *)pvTimerGetTimerID(timer);
    struct pivariety_cam *cam_pivariety = (struct pivariety_cam *)dev->priv;
#if PIVARIETY_EXPOSURE_TEST_EN_GAIN
    if (s_exp_v >= s_limited_abs_gain_index) {
        s_exp_add = false;
    } else if (s_exp_v < 10) {
        s_exp_add = true;
    }
    pivariety_set_total_gain_val(dev, s_exp_v);
    if (s_exp_add == true) {
        s_exp_v += 10;
    } else {
        s_exp_v -= 10;
    }
#else
    if (s_exp_v >= cam_pivariety->pivariety_para.exposure_max) {
        s_exp_add = false;
    } else if (s_exp_v < s_pivariety_exp_min) {
        s_exp_add = true;
    }
    pivariety_set_exp_val(dev, s_exp_v);
    if (s_exp_add == true) {
        s_exp_v += 100;
    } else {
        s_exp_v -= 100;
    }
#endif
    // ESP_LOGI(TAG, "E=%" PRIu32, s_exp_v);
}
#endif

static esp_err_t pivariety_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    esp_err_t ret = ESP_FAIL;
    ret = pivariety_write(dev->sccb_handle, STREAM_ON, enable ? 0x01 : 0x00);

    dev->stream_status = enable;
#if PIVARIETY_EXPOSURE_TEST_EN
    ae_timer_handle = xTimerCreate("AE_t", 100 / portTICK_PERIOD_MS, pdTRUE,
                                   (void *)dev, ae_timer_callback);
    xTimerStart(ae_timer_handle, portMAX_DELAY);
#endif
    ESP_LOGD(TAG, "Stream=%d", enable);
    return ret;
}

static esp_err_t pivariety_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    esp_err_t ret = ESP_OK;
    uint32_t regval;
    esp_cam_sensor_reg_val_t *sensor_reg;
    PIVARIETY_IO_MUX_LOCK(mux);

    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_HW_RESET:
        ret = pivariety_hw_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_SW_RESET:
        ret = pivariety_soft_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_S_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = pivariety_write(dev->sccb_handle, sensor_reg->regaddr, sensor_reg->value);
        break;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        ret = pivariety_set_stream(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
        ret = pivariety_set_test_pattern(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_G_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = pivariety_read(dev->sccb_handle, sensor_reg->regaddr, &regval);
        if (ret == ESP_OK) {
            sensor_reg->value = regval;
        }
        break;
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        ret = pivariety_get_sensor_id(dev, arg);
        break;
    default:
        break;
    }

    PIVARIETY_IO_MUX_UNLOCK(mux);
    return ret;
}

static esp_err_t pivariety_power_on(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        PIVARIETY_ENABLE_OUT_XCLK(dev->xclk_pin, dev->xclk_freq_hz);
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

static esp_err_t pivariety_power_off(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        PIVARIETY_DISABLE_OUT_XCLK(dev->xclk_pin);
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

static esp_err_t pivariety_delete(esp_cam_sensor_device_t *dev)
{
    ESP_LOGD(TAG, "del pivariety (%p)", dev);
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

static const esp_cam_sensor_ops_t pivariety_ops = {
    .query_para_desc = pivariety_query_para_desc,
    .get_para_value = pivariety_get_para_value,
    .set_para_value = pivariety_set_para_value,
    .query_support_formats = pivariety_query_support_formats,
    .query_support_capability = pivariety_query_support_capability,
    .set_format = pivariety_set_format,
    .get_format = pivariety_get_format,
    .priv_ioctl = pivariety_priv_ioctl,
    .del = pivariety_delete
};

esp_cam_sensor_device_t *pivariety_detect(esp_cam_sensor_config_t *config)
{
    esp_cam_sensor_device_t *dev = NULL;
    struct pivariety_cam *cam_pivariety;
    s_limited_abs_gain_index = ARRAY_SIZE(pivariety_abs_gain_val_map);
    if (config == NULL) {
        return NULL;
    }

    dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "No memory for camera");
        return NULL;
    }

    cam_pivariety = heap_caps_calloc(1, sizeof(struct pivariety_cam), MALLOC_CAP_DEFAULT);
    if (!cam_pivariety) {
        ESP_LOGE(TAG, "failed to calloc cam");
        free(dev);
        return NULL;
    }

    dev->name = (char *)PIVARIETY_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &pivariety_ops;
    dev->priv = cam_pivariety;
    dev->cur_format = &pivariety_format_info[CONFIG_CAMERA_PIVARIETY_MIPI_IF_FORMAT_INDEX_DEFAULT];

    // init para
    cam_pivariety->pivariety_para.exposure_val = dev->cur_format->isp_info->isp_v1_info.exp_def;
    cam_pivariety->pivariety_para.gain_index = dev->cur_format->isp_info->isp_v1_info.gain_def;
    cam_pivariety->pivariety_para.exposure_max = dev->cur_format->isp_info->isp_v1_info.vts - PIVARIETY_EXP_MAX_OFFSET;

    for (size_t i = 0; i < ARRAY_SIZE(pivariety_abs_gain_val_map); i++) {
        if (pivariety_abs_gain_val_map[i] > s_limited_abs_gain) {
            s_limited_abs_gain_index = i - 1;
            break;
        }
    }

    // Configure sensor power, clock, and SCCB port
    if (pivariety_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Camera power on failed");
        goto err_free_handler;
    }

    if (pivariety_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(TAG, "Get sensor ID failed");
        goto err_free_handler;
    } else if (dev->id.pid != PIVARIETY_PID) {
        ESP_LOGE(TAG, "Camera sensor is not PIVARIETY, PID=0x%x", dev->id.pid);
        goto err_free_handler;
    }
    ESP_LOGI(TAG, "Detected Camera sensor PID=0x%x", dev->id.pid);

    return dev;

err_free_handler:
    pivariety_power_off(dev);
    free(dev->priv);
    free(dev);

    return NULL;
}

#if CONFIG_CAMERA_PIVARIETY_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(pivariety_detect, ESP_CAM_SENSOR_MIPI_CSI, PIVARIETY_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return pivariety_detect(config);
}
#endif
