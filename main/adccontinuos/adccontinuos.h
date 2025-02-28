#ifndef ADC_CONTINUOS_H
#define ADC_CONTINUOS_H

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
#include "esp_adc/adc_continuous.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class AdcContinuos {
private:
    //adc_oneshot_unit_handle_t _adc1_handle, _adc2_handle;
  //  adc_continuous_handle_cfg_t adc_handle; 
    adc_continuous_handle_t _adc_handle = NULL;

    adc_continuous_config_t adc_config;
    adc_channel_t _adc_channels[6];
    uint16_t max_values[6], min_values[6];
    bool _invert = false;
  
    
public:
	AdcContinuos(adc_channel_t adc_channels[], bool invert);
	~AdcContinuos() {}
    void ConfigureAdc(bool (*callback)(adc_continuous_ctx_t *, const adc_continuous_evt_data_t *, void *));
    void Calibration();
    void ReadAdc(float (*read_data)[6]);
    void GetMinAndMaxValues();
   


};
#endif
#endif // ADC_CONTINUOS_H