#include "as5600.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include "driver/mcpwm_cap.h"

static const char *TAG = "AS5600";

As5600::As5600(gpio_num_t sda, gpio_num_t scl, i2c_port_num_t i2c_port, gpio_num_t gpio_pwm) {
    _gpio_pin_sda = sda;
    _gpio_pin_scl = scl;
    _i2c_port = i2c_port;
    _gpio_pwm = gpio_pwm;
}

void As5600::ConfigureAS5600(bool (*callback)(mcpwm_cap_channel_handle_t cap_chan,const mcpwm_capture_event_data_t *edata, void *user_data)) {
    // 1. Configurar o barramento I2C para o AS5600
	 _i2c_mst_config = {
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
	} else {
	    ESP_LOGE("I2C_CONFIG", "Falha ao escrever no registrador 0x08. Erro: %s", esp_err_to_name(err));
	}

    
    //2. Configurar PWM
    ESP_LOGI(TAG, "Inicializando MCPWM Capture para leitura do AS5600...");

    // 1. Cria e configura o timer de captura
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_timer_conf = {        
        .group_id = 0,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
  
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_timer_conf, &cap_timer));

    // 2. Cria o canal de captura
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = _gpio_pwm,
        .prescale = 1,                // Sem prescaler (máxima precisão)
        .flags {
			.pos_edge = true,       // Captura bordas de subida
            .neg_edge = true
         }      // Captura bordas de descida
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));
    
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, NULL));

    // 4. Habilita e inicia o timer/canal
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));
}

