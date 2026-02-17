#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif


#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include "driver/i2c_master.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"
#include "driver/mcpwm_cap.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class As5600 {
	
private:
    gpio_num_t _gpio_pwm;
	
public:
    As5600(gpio_num_t gpio_pwm);
    void ConfigureAS5600(mcpwm_cap_timer_handle_t cap_timer,
                         bool (*callback)(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data));


};


#endif 
#endif // AS5600_H