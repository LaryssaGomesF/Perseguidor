#include "encoder.h"
#include "driver/pulse_cnt.h"


Encoder::Encoder(int channelA, int channelB, bool invert, int low_limit, int high_limit){
	_channelA = channelA;
	_channelB = channelB;
	_invert = invert;
	_high_limit = high_limit;
	_low_limit = low_limit;
}

void Encoder::ConfigureEncoder(){
	pcnt_unit_config_t unit_config = {
	    .low_limit = _low_limit,
	    .high_limit = _high_limit,
	};
	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &_pcnt_unit));
	pcnt_chan_config_t chan_config = {
	    .edge_gpio_num = _channelA,
	    .level_gpio_num = _channelB,
	};
	pcnt_channel_handle_t pcnt_chan = NULL;
	ESP_ERROR_CHECK(pcnt_new_channel(_pcnt_unit, &chan_config, &pcnt_chan));
}

void Encoder::ReadEncoder(int* pulse_count ){
	 ESP_ERROR_CHECK(pcnt_unit_get_count(_pcnt_unit, pulse_count));
}