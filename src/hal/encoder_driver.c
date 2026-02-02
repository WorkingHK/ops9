#include "encoder_driver.h"
#include "config.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

static const char *TAG = "ENC_DRV";
static pcnt_unit_handle_t pcnt_unit = NULL;

esp_err_t encoder_driver_init(void)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_PIN_A,
        .level_gpio_num = ENCODER_PIN_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_PIN_B,
        .level_gpio_num = ENCODER_PIN_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

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

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    ESP_LOGI(TAG, "Encoder driver initialized on GPIO%d/%d", ENCODER_PIN_A, ENCODER_PIN_B);
    return ESP_OK;
}

int32_t encoder_driver_get_count(void)
{
    int count = 0;
    if (pcnt_unit) {
        pcnt_unit_get_count(pcnt_unit, &count);
    }
    return count;
}

void encoder_driver_reset(void)
{
    if (pcnt_unit) {
        pcnt_unit_clear_count(pcnt_unit);
    }
}
