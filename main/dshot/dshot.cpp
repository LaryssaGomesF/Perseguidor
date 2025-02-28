#include "dshot.h"
#include <cstdlib>
#define DSHOT_ESC_RESOLUTION_HZ 40000000
// Construtor da classe Dshot
Dshot::Dshot(gpio_num_t pin, bool direction, uint16_t limit) {
    _gpio_pin = pin;
    _direction = direction;
    _limit = limit;
}

void Dshot::ConfigureDshot() {
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = _gpio_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 10  // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &_esc_chan));

    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50 // extra delay between each frame
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &_dshot_encoder));

    ESP_ERROR_CHECK(rmt_enable(_esc_chan));

    _tx_config = {
        .loop_count = -1 // infinite loop
    };
    _throttle = {
        .throttle = 0,
        .telemetry_req = false // telemetry is not supported in this example
    };

    ESP_ERROR_CHECK(rmt_transmit(_esc_chan, _dshot_encoder, &_throttle, sizeof(_throttle), &_tx_config));
}

void Dshot::UpdateThrottle(int value) {	
	if(!_direction) {
		value = -value;
	}
	if(value < 0) {
		if(value < -_limit) {
			value = -_limit;
		}
		value = 48 - value;
	} else {
		if(value >= _limit) {
			value = _limit;
		}
		value = 1048 + value;
	}
    _throttle.throttle = value;
    ESP_ERROR_CHECK(rmt_transmit(_esc_chan, _dshot_encoder, &_throttle, sizeof(_throttle), &_tx_config));
    // the previous loop transfer is till undergoing, we need to stop it and restart,
    // so that the new throttle can be updated on the output
    ESP_ERROR_CHECK(rmt_disable(_esc_chan));
    ESP_ERROR_CHECK(rmt_enable(_esc_chan));
}