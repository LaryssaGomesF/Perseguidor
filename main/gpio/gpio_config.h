#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include <sys/_stdint.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Gpio {
private:
	gpio_num_t _gpio_num;
	
public:
    Gpio(gpio_num_t pin) {
		_gpio_num = pin;
	}
	~Gpio() {}
	void ConfigureGPIO();
	void UpdateGPIO(bool);
};
#endif
#endif 