
#ifndef ENCODER_H
#define ENCODER_H

#include "driver/adc_types_legacy.h"
#include "hal/adc_types.h"
#include <stdint.h>
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
#include "driver/pulse_cnt.h"


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Encoder {
private:
   	int _channelA, _channelB, _low_limit, _high_limit;
    bool _invert = false;
    pcnt_unit_handle_t _pcnt_unit = NULL;
    
public:
	Encoder(int channelA, int channelB, bool invert, int low_limit = -1000 , int high_limit = 1000);
	~Encoder() {}
    void ConfigureEncoder();
    void ReadEncoder(int* pulse_count);
};
#endif
#endif 