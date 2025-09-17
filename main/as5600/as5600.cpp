#include "as5600.h"
#include "esp_log.h"
#include "soc/gpio_num.h"

static const char *TAG = "AS5600";

As5600::As5600(gpio_num_t sda, gpio_num_t scl, i2c_port_num_t i2c_port) {
    _gpio_pin_sda = sda;
    _gpio_pin_scl = scl;
    _i2c_port = i2c_port;
}

void As5600::ConfigureAS5600() {
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
}

uint16_t As5600::ReadRawAngle() {
  // O que queremos escrever: o endereço do registrador de ângulo (0x0E)
    uint8_t reg_to_read = 0x0E;

    // Onde vamos armazenar a leitura: um buffer de 2 bytes
    uint8_t read_buffer[2];

    // Executa a operação combinada: Escreve 1 byte e depois lê 2 bytes
    esp_err_t err = i2c_master_transmit_receive(
        _dev_handle,          // Handle do dispositivo I2C
        &reg_to_read,         // Ponteiro para os dados a serem ESCRITOS
        1,                    // Número de bytes a ESCREVER
        read_buffer,          // Ponteiro para o buffer onde os dados serão LIDOS
        2,                    // Número de bytes a LER
        1000 / portTICK_PERIOD_MS // Timeout
    );

    // Verifique se a transação foi bem-sucedida
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha na transacao I2C (transmit_receive). Erro: %s", esp_err_to_name(err));
        // Retorna 0 ou um valor de erro. 0 é seguro pois é um ângulo válido,
        // mas para depuração, um valor como 0xFFFF (ou 4095) pode ser melhor.
        return 0xFFFF; 
    }

    // Combine os dois bytes para formar o valor de 12 bits
    uint16_t angle = ((uint16_t)read_buffer[0] << 8) | read_buffer[1];

    // Aplique a máscara para garantir que temos apenas 12 bits (0 a 4095)
    angle &= 0x0FFF;

    return angle;
}
