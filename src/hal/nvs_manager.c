#include "nvs_manager.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "NVS_MGR";
static const char *NVS_NAMESPACE = "ops9";

esp_err_t nvs_manager_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "NVS initialized");
    return ESP_OK;
}

esp_err_t nvs_manager_load_calibration(calibration_t *cal)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No calibration found");
        cal->valid = false;
        return err;
    }

    size_t required_size = sizeof(calibration_t);
    err = nvs_get_blob(handle, "calibration", cal, &required_size);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Calibration loaded");
        cal->valid = true;
    } else {
        ESP_LOGW(TAG, "Failed to load calibration");
        cal->valid = false;
    }

    return err;
}

esp_err_t nvs_manager_save_calibration(const calibration_t *cal)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(handle, "calibration", cal, sizeof(calibration_t));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
        ESP_LOGI(TAG, "Calibration saved");
    }

    nvs_close(handle);
    return err;
}

esp_err_t nvs_manager_clear_calibration(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_key(handle, "calibration");
    if (err == ESP_OK) {
        err = nvs_commit(handle);
        ESP_LOGI(TAG, "Calibration cleared");
    }

    nvs_close(handle);
    return err;
}
