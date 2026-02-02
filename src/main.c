#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "OPS9";

void app_main(void)
{
    ESP_LOGI(TAG, "OPS9 Positioning System Starting...");

    while(1) {
        ESP_LOGI(TAG, "Hello from OPS9");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
