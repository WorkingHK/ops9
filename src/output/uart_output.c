#include "uart_output.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "UART_OUT";
static position_state_t *g_position_state = NULL;

// Binary packet structure
typedef struct __attribute__((packed)) {
    uint8_t header[2];      // 0xAA 0x55
    uint8_t length;         // Payload length
    uint32_t timestamp_us;
    float x;
    float y;
    float heading;
    float confidence;
    uint8_t flags;          // bit 0: fail_safe, bit 1: calibration_valid
    uint8_t checksum;
} uart_packet_t;

static uint8_t calculate_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

static void uart_output_loop(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SYSTEM_UPDATE_PERIOD_MS);

    // Ensure period is at least 1 tick to avoid assertion failure
    if (period == 0) {
        ESP_LOGE(TAG, "Invalid period: SYSTEM_UPDATE_PERIOD_MS=%d results in 0 ticks", SYSTEM_UPDATE_PERIOD_MS);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UART output task loop started (200Hz, period=%d ticks)", period);

    while (1) {
        uart_packet_t packet;
        packet.header[0] = 0xAA;
        packet.header[1] = 0x55;
        packet.length = sizeof(uart_packet_t) - 4;  // Exclude header, length, checksum

        // Read position state
        if (xSemaphoreTake(g_position_state->mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            packet.timestamp_us = (uint32_t)g_position_state->timestamp_us;
            packet.x = g_position_state->x;
            packet.y = g_position_state->y;
            packet.heading = g_position_state->heading;
            packet.confidence = g_position_state->confidence;
            packet.flags = (g_position_state->fail_safe_mode ? 0x01 : 0x00);
            xSemaphoreGive(g_position_state->mutex);
        }

        // Calculate checksum
        packet.checksum = calculate_checksum((uint8_t*)&packet, sizeof(packet) - 1);

        // Send packet
        uart_write_bytes(UART_OUTPUT_NUM, (const char*)&packet, sizeof(packet));

        vTaskDelayUntil(&last_wake_time, period);
    }
}

esp_err_t uart_output_start(position_state_t *position_state)
{
    g_position_state = position_state;

    xTaskCreatePinnedToCore(uart_output_loop, "uart_output", 2048, NULL, 3, NULL, 1);
    ESP_LOGI(TAG, "UART output task started on core 1");
    return ESP_OK;
}
