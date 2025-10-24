#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include <sys/_stdint.h>
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
    // --- Configuração do Barramento I2C ---
    i2c_master_bus_config_t _i2c_mst_config;
    i2c_master_bus_handle_t _bus_handle;
    i2c_master_dev_handle_t _dev_handle;
    
    // --- Pinos e Porta I2C ---
    gpio_num_t _gpio_pin_sda;
    gpio_num_t _gpio_pin_scl;
    i2c_port_num_t _i2c_port;
	gpio_num_t _gpio_pwm;
    // --- Variáveis do Sensor ---
    const uint8_t _sensor_address = 0x36;
    const uint8_t _reg_raw_angle = 0x0E;
    const uint8_t _reg_magnitude = 0x1B;

public:

    As5600(gpio_num_t sda, gpio_num_t scl, i2c_port_num_t i2c_port, gpio_num_t gpio_pwm);
    ~As5600() {}

    void ConfigureAS5600(bool (*callback)(mcpwm_cap_channel_handle_t cap_chan,const mcpwm_capture_event_data_t *edata, void *user_data));

};

#endif 
#endif // AS5600_H