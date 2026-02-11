#include "imu_driver.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "IMU_DRV";

esp_err_t imu_driver_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_IMU_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_IMU_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_IMU_NUM, UART_IMU_TX_PIN, UART_IMU_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_IMU_NUM, 1024, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "IMU UART initialized on UART%d (RX:%d, TX:%d, baud:%d)",
             UART_IMU_NUM, UART_IMU_RX_PIN, UART_IMU_TX_PIN, UART_IMU_BAUD);
    return ESP_OK;
}

int imu_driver_read(uint8_t *buffer, size_t length)
{
    return uart_read_bytes(UART_IMU_NUM, buffer, length, pdMS_TO_TICKS(1));
}

bool imu_driver_parse_packet(const uint8_t *data, size_t len,
                              float *omega_x, float *omega_y, float *omega_z,
                              float *accel_x, float *accel_y, float *accel_z,
                              float *temperature)
{
    // WIT protocol parsing - simplified version
    // Full implementation should handle all packet types (0x51, 0x52, 0x53, etc.)
    if (len < 11 || data[0] != 0x55) {
        return false;
    }

    // Parse angular velocity packet (0x52)
    if (data[1] == 0x52) {
        int16_t wx = (int16_t)(data[3] << 8 | data[2]);
        int16_t wy = (int16_t)(data[5] << 8 | data[4]);
        int16_t wz = (int16_t)(data[7] << 8 | data[6]);

        *omega_x = wx / 32768.0f * 2000.0f * 3.14159f / 180.0f;  // Convert to rad/s
        *omega_y = wy / 32768.0f * 2000.0f * 3.14159f / 180.0f;
        *omega_z = -(wz / 32768.0f * 2000.0f * 3.14159f / 180.0f);  // Inverted for correct direction
        return true;
    }

    // Parse acceleration packet (0x51)
    if (data[1] == 0x51) {
        int16_t ax = (int16_t)(data[3] << 8 | data[2]);
        int16_t ay = (int16_t)(data[5] << 8 | data[4]);
        int16_t az = (int16_t)(data[7] << 8 | data[6]);

        *accel_x = ax / 32768.0f * 16.0f * 9.8f;  // Convert to m/s²
        *accel_y = ay / 32768.0f * 16.0f * 9.8f;
        *accel_z = az / 32768.0f * 16.0f * 9.8f;
        return true;
    }

    // Temperature parsing (0x40)
    if (data[1] == 0x40) {
        int16_t temp = (int16_t)(data[3] << 8 | data[2]);
        *temperature = temp / 100.0f;
        return true;
    }

    return false;
}
