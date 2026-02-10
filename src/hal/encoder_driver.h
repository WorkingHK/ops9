#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>
#include "esp_err.h"

// Encoder identifier
typedef enum {
    ENCODER_X = 0,  // Right wheel, vertical, measures X
    ENCODER_Y = 1   // Left wheel, horizontal, measures Y
} encoder_id_t;

// Initialize encoder pulse counter
esp_err_t encoder_driver_init(encoder_id_t encoder_id, int pin_a, int pin_b);

// Get current pulse count for specified encoder
int32_t encoder_driver_get_count(encoder_id_t encoder_id);

// Reset pulse counter for specified encoder
void encoder_driver_reset(encoder_id_t encoder_id);

#endif // ENCODER_DRIVER_H
