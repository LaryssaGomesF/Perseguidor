#ifndef ADC_H
#define ADC_H

#include "driver/adc_types_legacy.h"
#include "hal/adc_types.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Adc {
private:
    adc_oneshot_unit_handle_t _adc1_handle, _adc2_handle;
    adc_channel_t _adc1_channels[6], _adc2_channels[3];
    uint16_t max_values[9], min_values[9];
    bool _invert = false;
    
public:
	Adc(adc_channel_t adc1_channels[], adc_channel_t adc2_channels[], bool invert);
	~Adc() {}
    void ConfigureAdc();
    void Calibration(uint8_t);
    float ReadAdc(uint8_t);
};
#endif
#endif // ADC_H