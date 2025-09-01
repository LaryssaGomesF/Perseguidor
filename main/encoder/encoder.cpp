#include "encoder.h"
#include "driver/pulse_cnt.h"
#include <cstdio>


Encoder::Encoder(int channelA, int channelB, bool invert, int low_limit, int high_limit){
	_channelA = channelA;
	_channelB = channelB;
	_invert = invert;
	_high_limit = high_limit;
	_low_limit = low_limit;
}



void Encoder::ConfigureEncoder(){
    // Criação da unidade PCNT
    pcnt_unit_config_t unit_config = {
        .low_limit = _low_limit,
        .high_limit = _high_limit,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &_pcnt_unit));

    // Canal A
    pcnt_chan_config_t chanA_config = {
        .edge_gpio_num = _channelA,
        .level_gpio_num = _channelB, // importante para quadratura
    };
    
    pcnt_channel_handle_t _pcnt_chanA = NULL;
    pcnt_channel_handle_t _pcnt_chanB = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(_pcnt_unit, &chanA_config, &_pcnt_chanA));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(_pcnt_chanA, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(_pcnt_chanA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Canal B (inverso do anterior)
    pcnt_chan_config_t chanB_config = {
        .edge_gpio_num = _channelB,
        .level_gpio_num = _channelA,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(_pcnt_unit, &chanB_config, &_pcnt_chanB));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(_pcnt_chanB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(_pcnt_chanB, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    // Ativa e inicia
    ESP_ERROR_CHECK(pcnt_unit_enable(_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(_pcnt_unit));
}


void Encoder::ReadEncoder(int* pulse_count ){
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(_pcnt_unit, &count));
    *pulse_count = count;
    printf("Encoder count: %d\n", count);
}
