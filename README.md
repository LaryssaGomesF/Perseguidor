ESP-IDF template app
====================

This is a template application to be used with [Espressif IoT Development Framework](https://github.com/espressif/esp-idf).

Please check [ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for getting started instructions.

*Code in this repository is in the Public Domain (or CC0 licensed, at your option.)
Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.*


   // 1. Configurar o barramento I2C para o AS5600
	/* _i2c_mst_config = {
	    .i2c_port = _i2c_port,
	    .sda_io_num = _gpio_pin_sda,
	    .scl_io_num = _gpio_pin_scl,
	    .clk_source = I2C_CLK_SRC_DEFAULT,
	      .flags {
			1
		}
	};

    ESP_ERROR_CHECK(i2c_new_master_bus(&_i2c_mst_config, &_bus_handle));

    // 2. Configurar o dispositivo AS5600
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x36,
        .scl_speed_hz = 5000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(_bus_handle, &dev_cfg, &_dev_handle));


    ESP_LOGI(TAG, "AS5600 configurado na porta I2C %d.", _i2c_port);
    
	   // 1. Definir o endereço do registrador que vamos escrever.
	uint8_t reg_to_write = 0x08;
	
	// 2. Construir o valor para o registrador 0x08.
	//    OUTS(5:4) = 10 (digital PWM)
	//    PWMF(7:6) = 11 (920 Hz)
	uint8_t config_value = (0b10 << 4) | (0b11 << 6); // Resultado: 0xE0
	
	// 3. Preparar o buffer de transmissão.
	//    Formato: [endereço_do_registrador, dado_a_escrever]
	uint8_t write_buffer[2];
	write_buffer[0] = reg_to_write;   // Byte 0: Endereço do registrador (0x08)
	write_buffer[1] = config_value;   // Byte 1: Dado para o registrador 0x08 (0xE0)
	
	// 4. Chamar a função i2c_master_transmit.
	esp_err_t err = i2c_master_transmit(
	    _dev_handle,          // O handle do seu dispositivo I2C
	    write_buffer,        // O buffer que acabamos de criar
	    sizeof(write_buffer),// O tamanho total do buffer (2 bytes)
	    pdMS_TO_TICKS(1000)  // Timeout de 1000 ms
	);
	
	
	// 5. Verificar se a escrita foi bem-sucedida.
	if (err == ESP_OK) {
	    ESP_LOGI("I2C_CONFIG", "Registrador 0x08 configurado com sucesso com o valor 0x%02X", config_value);
	    
	    uint8_t write_buffer_set[2];
		write_buffer_set[0] = 0xFF;   
		write_buffer_set[1] = 0x40;   
	    esp_err_t err1 = i2c_master_transmit(
	    _dev_handle,          // O handle do seu dispositivo I2C
	    write_buffer_set,        // O buffer que acabamos de criar
	    sizeof(write_buffer_set),// O tamanho total do buffer (2 bytes)
	    pdMS_TO_TICKS(1000)  // Timeout de 1000 ms
		);
		if (err1 == ESP_OK) {
			ESP_LOGI("I2C_CONFIG", "Registrador 0xFF configurado com sucesso com o valor 0x40");
		}else{
			ESP_LOGE("I2C_CONFIG", "Falha ao realizar BURN no registrador 0xFF. Erro: %s", esp_err_to_name(err));
		}
	} else {
	    ESP_LOGE("I2C_CONFIG", "Falha ao escrever no registrador 0x08. Erro: %s", esp_err_to_name(err));
	}

    */