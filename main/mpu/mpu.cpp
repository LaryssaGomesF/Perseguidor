#include "mpu.h"
#include <math.h> 
const float GYRO_SENSITIVITY = 131.0f;
const float ACCEL_SENSITIVITY = 16384.0f;

Mpu::Mpu(gpio_num_t sda, gpio_num_t scl, i2c_port_num_t i2c_port){
	_i2c_port = i2c_port;
	_gpio_pin_sda = sda ;
	_gpio_pin_scl = scl;
	
	  // Inicializa as variáveis de calibração
    _gyro_x_cal_sum = 0;
    _gyro_y_cal_sum = 0;
    _gyro_z_cal_sum = 0;
    _gyro_x_offset = 0.0f;
    _gyro_y_offset = 0.0f;
    _gyro_z_offset = 0.0f;
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
    
    // 2. Configurar faixa do acelerômetro para ±2g (ACCEL_CONFIG = 0x1C)
	uint8_t accel_config[2] = {0x1C, 0x00};
	i2c_master_transmit(_dev_handle, accel_config, 2, 50);

	// 3. Configurar faixa do giroscópio para ±250°/s (GYRO_CONFIG = 0x1B)
	uint8_t gyro_config[2] = {0x1B, 0x00};
	i2c_master_transmit(_dev_handle, gyro_config, 2, 50);
}


void Mpu::ReadMPU(int* gyroX, int* gyroY, int* gyroZ, int* accelX, int* accelY, int* accelZ, bool fixOffset) {
 	
	 for(uint8_t i=0; i<6; i+=2) {
			i2c_master_transmit_receive(_dev_handle, &_gyro_data_address[i], 1, &_gyro_data_read[i], 1, 50);
			i2c_master_transmit_receive(_dev_handle, &_gyro_data_address[i+1], 1, &_gyro_data_read[i+1], 1, 50);
			
			i2c_master_transmit_receive(_dev_handle, &_acel_data_address[i], 1, &_acel_data_read[i], 1, 50);
			i2c_master_transmit_receive(_dev_handle, &_acel_data_address[i+1], 1, &_acel_data_read[i+1], 1, 50);
			
			_gyro_combined_values[i/2] = (int16_t)((_gyro_data_read[i] << 8) | _gyro_data_read[i+1]);
			_acel_combined_values[i/2] = (int16_t)((_acel_data_read[i] << 8) | _acel_data_read[i+1]);
	}
	
	
	// Aplicar antes a correção
	  if(fixOffset){
        *gyroX  = _gyro_combined_values[0] - _gyro_x_offset;
        *gyroY  = _gyro_combined_values[1] - _gyro_y_offset;
        *gyroZ  = _gyro_combined_values[2] - _gyro_z_offset;

        *accelX = _acel_combined_values[0] - _accel_x_offset;
        *accelY = _acel_combined_values[1] - _accel_y_offset;
        *accelZ = _acel_combined_values[2] - _accel_z_offset;
    } else {
        *gyroX  = _gyro_combined_values[0];
        *gyroY  = _gyro_combined_values[1];
        *gyroZ  = _gyro_combined_values[2];

        *accelX = _acel_combined_values[0];
        *accelY = _acel_combined_values[1];
        *accelZ = _acel_combined_values[2];
    }
	
	printf("Gyro - X: %05d  /  Y: %05d  /  Z: %05d  \nAcel - X: %05d  /  Y: %05d  /  Z: %05d  \n",
           *gyroX, *gyroY, *gyroZ, *accelX, *accelY, *accelZ);
}



void Mpu::AccumulateGyroSample() {
    int gx, gy, gz, ax, ay, az;
    // Lê os dados brutos (sem aplicar correção ainda)
    ReadMPU(&gx, &gy, &gz, &ax, &ay, &az);
    
    _gyro_x_cal_sum += gx;
    _gyro_y_cal_sum += gy;
    _gyro_z_cal_sum += gz;
    
    _accel_x_cal_sum += ax;
    _accel_y_cal_sum += ay;
    _accel_z_cal_sum += az;
    
}

void Mpu::ComputeAndSetOffsets(int total_samples) {
    if (total_samples > 0) {
        _gyro_x_offset = (float)_gyro_x_cal_sum / total_samples;
        _gyro_y_offset = (float)_gyro_y_cal_sum / total_samples;
        _gyro_z_offset = (float)_gyro_z_cal_sum / total_samples;
        _accel_x_offset = (float)_accel_x_cal_sum / total_samples;
        _accel_y_offset = (float)_accel_y_cal_sum / total_samples;
        _accel_z_offset = (float)_accel_z_cal_sum / total_samples;
    }
    printf("MPU_CAL Calibracao concluida\n");
    printf("Offsets Gyro: X:%.2f, Y:%.2f, Z:%.2f\n", _gyro_x_offset, _gyro_y_offset, _gyro_z_offset);
    printf("Offsets Accel: X:%.2f, Y:%.2f, Z:%.2f\n", _accel_x_offset, _accel_y_offset, _accel_z_offset);
}

void Mpu::GetFilteredAngles(float* roll, float* pitch, float* yaw, float dt) {
    int gx, gy, gz, ax, ay, az;
    
    // 1. Lê os dados brutos (usando sua função ReadMPU ou a _readRawData)
    // Assumindo que ReadMPU já aplica a calibração de offset do giroscópio.
    ReadMPU(&gx, &gy, &gz, &ax, &ay, &az);

    // -------------------------------------------------
    // PARTE 1: CÁLCULO DE ROLL E PITCH (Lógica Existente)
    // -------------------------------------------------
    
    // Calcula o ângulo do acelerômetro (com a proteção de magnitude que implementamos)
    float accel_roll = 0.0f;
    float accel_pitch = 0.0f;
    float accel_magnitude = sqrt((float)ax * ax + (float)ay * ay + (float)az * az);
    const float tolerance = 0.15f; 

    if (accel_magnitude > ACCEL_SENSITIVITY * (1 - tolerance) && 
        accel_magnitude < ACCEL_SENSITIVITY * (1 + tolerance)) 
    {
        accel_roll = atan2(ay, az) * 180.0 / M_PI;
        accel_pitch = atan2(-ax, sqrt((float)ay * ay + (float)az * az)) * 180.0 / M_PI;
    }

    // Converte a velocidade angular do giroscópio para graus/segundo
    float gyro_rate_x = (float)gx / GYRO_SENSITIVITY;
    float gyro_rate_y = (float)gy / GYRO_SENSITIVITY;

    // Executa o filtro de Kalman para Roll e Pitch
    *roll = kalmanX.update(accel_roll, gyro_rate_x, dt);
    *pitch = kalmanY.update(accel_pitch, gyro_rate_y, dt);

    // -------------------------------------------------
    // PARTE 2: CÁLCULO DE YAW (Nova Lógica)
    // -------------------------------------------------

    // Converte a velocidade angular do giroscópio Z para graus/segundo
    float gyro_rate_z = (float)gz / GYRO_SENSITIVITY;

    // Integra a velocidade angular para obter o ângulo de Yaw
    // Acumulamos o valor na variável de membro da classe.
    _yawAngle += gyro_rate_z * dt;

    // Atribui o valor acumulado ao ponteiro de saída
    *yaw = _yawAngle;
}
