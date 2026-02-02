#ifndef CONFIG_H
#define CONFIG_H

// System Configuration
#define SYSTEM_UPDATE_RATE_HZ 200
#define SYSTEM_UPDATE_PERIOD_MS (1000 / SYSTEM_UPDATE_RATE_HZ)

// UART Configuration
#define UART_IMU_NUM UART_NUM_1
#define UART_IMU_TX_PIN 17
#define UART_IMU_RX_PIN 16
#define UART_IMU_BAUD 115200

#define UART_OUTPUT_NUM UART_NUM_0
#define UART_OUTPUT_BAUD 115200

// Encoder Configuration
#define ENCODER_PCNT_UNIT PCNT_UNIT_0
#define ENCODER_PIN_A 25
#define ENCODER_PIN_B 26
#define ENCODER_PPR 2048  // AMT103 pulses per revolution

// WiFi Configuration
#define WIFI_DEBUG_ENABLED 1
#define WIFI_DEBUG_PORT 9090
#define WIFI_DEBUG_RATE_HZ 10

// Calibration
#define GYRO_BIAS_CAL_DURATION_S 30
#define GYRO_BIAS_CAL_SAMPLES (GYRO_BIAS_CAL_DURATION_S * SYSTEM_UPDATE_RATE_HZ)

// Fail-safe thresholds
#define IMU_TIMEOUT_MS 50
#define MAX_ANGULAR_VELOCITY 10.0f  // rad/s
#define MAX_ACCELERATION 50.0f      // m/s²

#endif // CONFIG_H
