#include "adc.h"
#include "driver/adc_types_legacy.h"
#include "hal/adc_types.h"

#define ADC_ATTEN 		ADC_ATTEN_DB_12

Adc::Adc(adc_channel_t adc1_channels[], adc_channel_t adc2_channels[], bool invert) {
	for(int i=0; i < sizeof(_adc1_channels) / sizeof(adc_channel_t); i++) {
		_adc1_channels[i] = adc1_channels[i];
	}
	for(int i=0; i < sizeof(_adc2_channels) / sizeof(adc_channel_t); i++) {
		_adc2_channels[i] = adc2_channels[i];
	}
	
	_invert = invert;
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif
    return calibrated;
}


void Adc::ConfigureAdc() {
	adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &_adc1_handle));
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &_adc2_handle));
    
    adc_oneshot_chan_cfg_t config = {
		.atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };    
    for(int i=0; i < sizeof(_adc1_channels) / sizeof(adc_channel_t); i++) {
		ESP_ERROR_CHECK(adc_oneshot_config_channel(_adc1_handle, _adc1_channels[i], &config));
		adc_cali_handle_t adc1_cali_handle = NULL;
		bool do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, _adc1_channels[i], ADC_ATTEN, &adc1_cali_handle);
	}
     for(int i=0; i < sizeof(_adc2_channels) / sizeof(adc_channel_t); i++) {
		ESP_ERROR_CHECK(adc_oneshot_config_channel(_adc2_handle, _adc2_channels[i], &config));
		adc_cali_handle_t adc2_cali_handle = NULL;
		bool do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_2, _adc2_channels[i], ADC_ATTEN, &adc2_cali_handle);
	}
	
	for(uint8_t i = 0; i < 9; i++) {
		max_values[i] = 0;
		min_values[i] = 9999;
	}
}

void Adc::Calibration(uint8_t i) {
	int result;
	if (i >= 0 && i < 6) {
		ESP_ERROR_CHECK(adc_oneshot_read(_adc1_handle, _adc1_channels[i], &result));
	} else {
		ESP_ERROR_CHECK(adc_oneshot_read(_adc2_handle, _adc2_channels[i-6], &result));
	}
	if(result > max_values[i]) {
		max_values[i] = result;
	}
	if(result < min_values[i]) {
		min_values[i] = result;
	}
}

float Adc::ReadAdc(uint8_t i) {
	int result;
	if (i >= 0 && i < 6) {
		ESP_ERROR_CHECK(adc_oneshot_read(_adc1_handle, _adc1_channels[i], &result));
	} else {
		ESP_ERROR_CHECK(adc_oneshot_read(_adc2_handle, _adc2_channels[i-6], &result));
	}
	float value = 10.0*((float)(result - min_values[i]) / (float)(max_values[i] - min_values[i]));
	if (_invert) {
		value = 10.0 - value;
	}
	
	return value;
}