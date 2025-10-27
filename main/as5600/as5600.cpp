#include "as5600.h"
#include "esp_log.h"
#include "driver/mcpwm_cap.h"

static const char *TAG = "AS5600";

As5600::As5600(gpio_num_t gpio_pwm) {
    _gpio_pwm = gpio_pwm;
}

void As5600::ConfigureAS5600(mcpwm_cap_timer_handle_t cap_timer,
                             bool (*callback)(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)) {

    ESP_LOGI(TAG, "Inicializando MCPWM Capture para leitura do AS5600 (GPIO %d)...", _gpio_pwm);

    // 1. Cria o canal de captura (sem criar novo timer!)
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = _gpio_pwm,
        .prescale = 1,
        .flags = {
            .pos_edge = true,
            .neg_edge = true
        }
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    // 2. Registra o callback
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, NULL));

    // 3. Habilita o canal
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));
}
