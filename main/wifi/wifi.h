#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_system.h"


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Wifi {
private:
	wifi_init_config_t cfg ;
	wifi_config_t wifi_config;
 
	
public:
 	 Wifi();

	void ConfigureWifi();
	void ConnectWifi(const char* ssid, const char* password);
};
#endif
#endif 