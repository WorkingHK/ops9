#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// WIT protocol packet structure
typedef struct {
    uint8_t header[2];  // 0x55 0x5X
    uint8_t data[8];
    uint8_t checksum;
} wit_packet_t;

// Initialize IMU UART driver
esp_err_t imu_driver_init(void);

// Read raw data from IMU (non-blocking)
int imu_driver_read(uint8_t *buffer, size_t length);

// Parse WIT protocol packet
bool imu_driver_parse_packet(const uint8_t *data, size_t len,
                              float *omega_x, float *omega_y, float *omega_z,
                              float *accel_x, float *accel_y, float *accel_z,
                              float *temperature);

#endif // IMU_DRIVER_H
