#include "gpio_config.h"

void Gpio::ConfigureGPIO() {
    gpio_config_t io_conf = {};
    
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << _gpio_num);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    
    gpio_config(&io_conf);
}

void Gpio::UpdateGPIO(bool level) {
	if (level) 
		gpio_set_level(_gpio_num, 1);  
	else 
    	gpio_set_level(_gpio_num, 0);
}