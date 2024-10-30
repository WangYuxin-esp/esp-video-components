/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_xclk_generator.h"

#include "unity.h"
#include "unity_test_utils.h"
#include "unity_test_utils_memory.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_TIMER_CLK_CFG      LEDC_AUTO_CLK
#define LEDC_FREQUENCY          (10000000) // Frequency in Hertz. Set frequency at 10MHz
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO

#define TEST_MEMORY_LEAK_THRESHOLD (-100)

static size_t before_free_8bit;
static size_t before_free_32bit;

static void check_leak(size_t before_free, size_t after_free, const char *type)
{
    ssize_t delta = after_free - before_free;
    printf("MALLOC_CAP_%s: Before %u bytes free, After %u bytes free (delta %d)\n", type, before_free, after_free, delta);
    TEST_ASSERT_MESSAGE(delta >= TEST_MEMORY_LEAK_THRESHOLD, "memory leak");
}

void setUp(void)
{
    before_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    before_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
}

void tearDown(void)
{
    size_t after_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t after_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
    check_leak(before_free_8bit, after_free_8bit, "8BIT");
    check_leak(before_free_32bit, after_free_32bit, "32BIT");
}

TEST_CASE("XCLK output operation", "[xclk_generator]")
{
    esp_xclk_generator_handle_t xclk_handle = NULL;
    for (int i = 0; i < 5; i++) {
        TEST_ESP_OK(xclk_ledc_generator_start(LEDC_TIMER, LEDC_TIMER_CLK_CFG, LEDC_CHANNEL, LEDC_FREQUENCY, LEDC_OUTPUT_IO, &xclk_handle));
        vTaskDelay(5 / portTICK_PERIOD_MS);
        TEST_ESP_OK(esp_xclk_generator_stop(xclk_handle));
    }
}

void app_main(void)
{
    /**
     * \ \     /_ _| __ \  ____|  _ \
     *  \ \   /   |  |   | __|   |   |
     *   \ \ /    |  |   | |     |   |
     *    \_/   ___|____/ _____|\___/
    */

    printf("\r\n");
    printf("\\ \\     /_ _| __ \\  ____|  _ \\  \r\n");
    printf(" \\ \\   /   |  |   | __|   |   |\r\n");
    printf("  \\ \\ /    |  |   | |     |   | \r\n");
    printf("   \\_/   ___|____/ _____|\\___/  \r\n");

    unity_run_menu();
}
