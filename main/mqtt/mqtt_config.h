#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif


#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include "mqtt_client.h"


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Mqtt_Config {
private:
	 esp_mqtt_client_handle_t _client;
	
public:
    Mqtt_Config() {}
	~Mqtt_Config() {}
	void ConfigureMqtt();
	void publishData(int16_t gyroX, int16_t gyroY, int16_t gyroZ, int16_t accelX, int16_t accelY, int16_t accelZ, float total_accumulated_degrees_right, float total_accumulated_degrees_left, uint32_t ts);
	void publishRaw(const char* payload);
};
#endif
#endif 