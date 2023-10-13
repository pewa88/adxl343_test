/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "adxl343.h"

#define CORE0   0
#define CORE1   1
static const char TAG[] = "main";
static adxl343_handle_t adxl343_handle;

void app_main(void)
{
    /* set loglevel to info globally */
    esp_log_level_set("*", ESP_LOG_INFO);

    ADXL343_Init(&adxl343_handle);

    /* Create task that reads temperature once per second */
    if (pdTRUE != xTaskCreatePinnedToCore(Adxl343_Task, "adxl343", 2048, &adxl343_handle, tskIDLE_PRIORITY+1U, NULL, CORE1))
    {
        ESP_LOGE(TAG, "creating ADXL343 task failed");
    }
}
