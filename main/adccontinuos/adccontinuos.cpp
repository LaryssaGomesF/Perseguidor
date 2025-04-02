#include "adccontinuos.h"
#include "driver/adc_types_legacy.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <cstdio>
#include <inttypes.h>




AdcContinuos::AdcContinuos(adc_channel_t adc_channels[], bool invert) {
	for(int i=0; i < sizeof(_adc_channels) / sizeof(adc_channel_t); i++) {
		_adc_channels[i] = adc_channels[i];
	}

	_invert = invert;
}


void AdcContinuos::ConfigureAdc(bool (*callback)(adc_continuous_ctx_t *, const adc_continuous_evt_data_t *, void *)) {

	adc_continuous_handle_cfg_t handle_config = {
		.max_store_buf_size = 24,
		.conv_frame_size = 12,	
	};
	ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &_adc_handle));
	
	adc_continuous_config_t adc_config = {
		.pattern_num = 6,
		.sample_freq_hz = 20*1000,
		.conv_mode = ADC_CONV_SINGLE_UNIT_1,
		.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
	
	};
	
	adc_digi_pattern_config_t adc_channel_config[SOC_ADC_PATT_LEN_MAX] = {};
	for (int i=0; i< 6; i++)
		{
			adc_channel_config[i].channel = _adc_channels[i];
			adc_channel_config[i].atten = ADC_ATTEN_DB_12;
			adc_channel_config[i].bit_width = ADC_BITWIDTH_12;
			adc_channel_config[i].unit = ADC_UNIT_1;
		}
	adc_config.adc_pattern = adc_channel_config;
	
	ESP_ERROR_CHECK(adc_continuous_config(_adc_handle, &adc_config));
	
	adc_continuous_evt_cbs_t cb_config = {
		.on_conv_done = callback,
	};
	ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(_adc_handle, &cb_config, NULL));
	
	ESP_ERROR_CHECK(adc_continuous_start(_adc_handle));
    
	for(uint8_t i = 0; i < 6; i++) {
		max_values[i] = 0;
		min_values[i] = 999;
	}
	
}

void AdcContinuos::Calibration() {	
	uint8_t buf[30];
	uint32_t rxLen = 0;
    adc_continuous_read(_adc_handle, buf, 12, &rxLen, 0);
	for (int i=0; i<rxLen; i+=SOC_ADC_DIGI_RESULT_BYTES){
			adc_digi_output_data_t *p = (adc_digi_output_data_t *)&buf[i];
			uint16_t channel = p->type1.channel;
			uint16_t data = p->type1.data;
			if (channel == ADC_CHANNEL_6){
				if(data> max_values[0]) {
					max_values[0] = data;
				}	
				if(data < min_values[0]) {
					min_values[0] = data;
				}
			}
	        if (channel == ADC_CHANNEL_7){
				if(data> max_values[1]) {
					max_values[1] = data;
				}	
				if(data < min_values[1]) {
					min_values[1] = data;
				}
			}
	        if (channel == ADC_CHANNEL_3){
				if(data> max_values[2]) {
					max_values[2] = data;
				}	
				if(data < min_values[2]) {
					min_values[2] = data;
				}
			}
	        if (channel == ADC_CHANNEL_4){
				if(data> max_values[3]) {
					max_values[3] = data;
				}	
				if(data < min_values[3]) {
					min_values[3] = data;
				}
			}
	        if (channel == ADC_CHANNEL_0){
				if(data> max_values[4]) {
					max_values[4] = data;
				}	
				if(data < min_values[4]) {
					min_values[4] = data;
				}
			}
	        if (channel == ADC_CHANNEL_5){
				if(data> max_values[5]) {
					max_values[5] = data;
				}	
				if(data < min_values[5]) {
					min_values[5] = data;
				}
			}		
	}
	
}

void AdcContinuos::GetMinAndMaxValues(){
	 printf("min: %hu ; %hu , %hu, %hu ,%hu, %hu \n", min_values[0],min_values[1],min_values[2],min_values[3],min_values[4],min_values[5]);
	 printf("max: %hu ; %hu , %hu, %hu ,%hu, %hu \n", max_values[0],max_values[1],max_values[2],max_values[3],max_values[4],max_values[5]);
}

void AdcContinuos::ReadAdc(float (*read_data)[6]) {
	uint8_t buf[30];
	uint32_t rxLen = 0;
    adc_continuous_read(_adc_handle, buf, 12, &rxLen, 0);
		for (int i=0; i<rxLen; i+=SOC_ADC_DIGI_RESULT_BYTES)
		{
			adc_digi_output_data_t *p = (adc_digi_output_data_t *)&buf[i];
			uint16_t channel = p->type1.channel;
			uint16_t data = p->type1.data;
			//CANAL 6
			if (channel == ADC_CHANNEL_6){
				float value = 10.0*((float)(data - min_values[0]) / (float)(max_values[0] - min_values[0]));
				if (_invert) {
					value = 10.0 - value;
				}
				(*read_data)[0] = value;
			}
			
			//CANAL 7 
	        if (channel == ADC_CHANNEL_7){
				float value = 10.0*((float)(data - min_values[1]) / (float)(max_values[1] - min_values[1]));
				if (_invert) {
					value = 10.0 - value;
				}
				(*read_data)[1] = value;			
			}
			
			//CANAL 3 
	        if (channel == ADC_CHANNEL_3){
				float value = 10.0*((float)(data - min_values[2]) / (float)(max_values[2] - min_values[2]));
				if (_invert) {
					value = 10.0 - value;
				}
				(*read_data)[2] = value;			
			}
			
			//CANAL 4
	        if (channel == ADC_CHANNEL_4){
				float value = 10.0*((float)(data - min_values[3]) / (float)(max_values[3] - min_values[3]));
				if (_invert) {
					value = 10.0 - value;
				}
				(*read_data)[3] = value;			
			}
			
			// CANAL 0 
	        if (channel == ADC_CHANNEL_0){
				float value = 10.0*((float)(data - min_values[4]) / (float)(max_values[4] - min_values[4]));
				if (_invert) {
					value = 10.0 - value;
				}
				(*read_data)[4] = value;			
			}
			
			// CANAL 5
	        if (channel == ADC_CHANNEL_5){
				float value = 10.0*((float)(data - min_values[5]) / (float)(max_values[5] - min_values[5]));
				if (_invert) {
					value = 10.0 - value;
				}
				(*read_data)[5] = value;			
			}
			
		}
		
}
