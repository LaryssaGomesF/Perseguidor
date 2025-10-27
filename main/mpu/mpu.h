#ifndef MPU_H
#define MPU_H

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
#include "kalmanfilter/kalmanfilter.h"

#ifdef __cplusplus
}

#endif

#ifdef __cplusplus

class Mpu {
private:
	i2c_master_bus_config_t _i2c_mst_config;
	i2c_master_bus_handle_t _bus_handle;
	i2c_device_config_t _dev_cfg;
	i2c_master_dev_handle_t _dev_handle;
	gpio_num_t _gpio_pin_sda;
	gpio_num_t _gpio_pin_scl;
	i2c_port_num_t _i2c_port;
	
	uint8_t _activate_mpu[2];
    
    uint8_t _gyro_data_address[6] = {0x43, 0x44, 0x45, 0x46, 0x47, 0x48};
    uint8_t _gyro_data_read[6];
    int16_t _gyro_combined_values[3];
    
    uint8_t _acel_data_address[6] = {0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40};
    uint8_t _acel_data_read[6];
    int16_t _acel_combined_values[3];

   
	
public:
    Mpu(gpio_num_t sda, gpio_num_t scl, i2c_port_num_t i2c_port);
	~Mpu() {}
	void ConfigureMPU();
	void ReadMPU(int* gyroX, int* gyroY, int* gyroZ, int* accelX, int* accelY, int* accelZ);

};
#endif
#endif 