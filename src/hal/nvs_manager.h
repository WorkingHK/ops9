#ifndef NVS_MANAGER_H
#define NVS_MANAGER_H

#include "types.h"
#include "esp_err.h"

// Initialize NVS
esp_err_t nvs_manager_init(void);

// Load calibration from NVS
esp_err_t nvs_manager_load_calibration(calibration_t *cal);

// Save calibration to NVS
esp_err_t nvs_manager_save_calibration(const calibration_t *cal);

// Clear calibration from NVS
esp_err_t nvs_manager_clear_calibration(void);

#endif // NVS_MANAGER_H
