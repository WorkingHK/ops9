#include "encoder_driver.h"
#include "config.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

static const char *TAG = "ENC_DRV";
static pcnt_unit_handle_t pcnt_units[2] = {NULL, NULL};

esp_err_t encoder_driver_init(encoder_id_t encoder_id, int pin_a, int pin_b)
{
    if (encoder_id >= 2) {
        ESP_LOGE(TAG, "Invalid encoder_id: %d", encoder_id);
        return ESP_ERR_INVALID_ARG;
    }

    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_units[encoder_id]));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = pin_a,
        .level_gpio_num = pin_b,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_units[encoder_id], &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = pin_b,
        .level_gpio_num = pin_a,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_units[encoder_id], &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a,
                                                  PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                  PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a,
                                                   PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                   PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b,
                                                  PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                  PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b,
                                                   PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                   PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_units[encoder_id]));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_units[encoder_id]));

    ESP_LOGI(TAG, "Encoder %s initialized on GPIO%d/%d",
             encoder_id == ENCODER_X ? "X" : "Y", pin_a, pin_b);
    return ESP_OK;
}

int32_t encoder_driver_get_count(encoder_id_t encoder_id)
{
    if (encoder_id >= 2 || pcnt_units[encoder_id] == NULL) {
        return 0;
    }

    int count = 0;
    pcnt_unit_get_count(pcnt_units[encoder_id], &count);
    return count;
}

void encoder_driver_reset(encoder_id_t encoder_id)
{
    if (encoder_id < 2 && pcnt_units[encoder_id] != NULL) {
        pcnt_unit_clear_count(pcnt_units[encoder_id]);
    }
}
