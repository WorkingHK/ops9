#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>
#include "esp_err.h"

// Initialize encoder pulse counter
esp_err_t encoder_driver_init(void);

// Get current pulse count
int32_t encoder_driver_get_count(void);

// Reset pulse counter
void encoder_driver_reset(void);

#endif // ENCODER_DRIVER_H
