#include "mpu.h"

Mpu::Mpu(gpio_num_t sda, gpio_num_t scl, i2c_port_num_t i2c_port){
	_i2c_port = i2c_port;
	_gpio_pin_sda = sda ;
	_gpio_pin_scl = scl;
}

void Mpu::ConfigureMPU() {	   
	
   _i2c_mst_config = {   	    	
	    .i2c_port = I2C_NUM_0,
	    .sda_io_num = GPIO_NUM_21,
	    .scl_io_num = GPIO_NUM_22,	   
    	.clk_source = I2C_CLK_SRC_DEFAULT,
	    .glitch_ignore_cnt = 7,
	    .flags {
			1
		}
	};
	
	
	
	ESP_ERROR_CHECK(i2c_new_master_bus(&_i2c_mst_config, &_bus_handle));
	
	i2c_device_config_t _dev_cfg = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address = 0x68,
	    .scl_speed_hz = 100000,
	};

	ESP_ERROR_CHECK(i2c_master_bus_add_device(_bus_handle, &_dev_cfg, &_dev_handle));
	
	_activate_mpu[0] = 0x6B;
    _activate_mpu[1] = 0x00;
    
    i2c_master_transmit(_dev_handle, _activate_mpu, 2, 50);
}

void Mpu::ReadMPU() {
 	
	 for(uint8_t i=0; i<6; i+=2) {
			i2c_master_transmit_receive(_dev_handle, &_gyro_data_address[i], 1, &_gyro_data_read[i], 1, 50);
			i2c_master_transmit_receive(_dev_handle, &_gyro_data_address[i+1], 1, &_gyro_data_read[i+1], 1, 50);
			
			i2c_master_transmit_receive(_dev_handle, &_acel_data_address[i], 1, &_acel_data_read[i], 1, 50);
			i2c_master_transmit_receive(_dev_handle, &_acel_data_address[i+1], 1, &_acel_data_read[i+1], 1, 50);
			
			_gyro_combined_values[i/2] = (int16_t)((_gyro_data_read[i] << 8) | _gyro_data_read[i+1]);
			_acel_combined_values[i/2] = (int16_t)((_acel_data_read[i] << 8) | _acel_data_read[i+1]);
	}
		
	printf("\nGyro - X: %05d  /  Y: %05d  /  Z: %05d  \nAcel - X: %05d  /  Y: %05d  /  Z: %05d  ", _gyro_combined_values[0], _gyro_combined_values[1], _gyro_combined_values[2], _acel_combined_values[0], _acel_combined_values[1], _acel_combined_values[2]);
}