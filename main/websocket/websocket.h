#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_websocket_client.h"
#include "esp_log.h"


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Websocket {
private:
	esp_websocket_client_handle_t client;
	esp_websocket_client_config_t websocket_cfg;
 
	
public:
 	Websocket();

	void ConfigureWebsocket(char *uri);
	void SendRawData(int gyroX, int gyroY, int gyroZ, int accelX, int accelY, int accelZ, int encoderR, int encoderL);
	


};
#endif
#endif 