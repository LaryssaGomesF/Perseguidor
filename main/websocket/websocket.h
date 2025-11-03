#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <vector>
#include <sys/_stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "esp_websocket_client.h"
#include "esp_log.h"


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

typedef struct {
	uint64_t timestamp_ms; 
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    float total_accumulated_turns_right;
    float total_accumulated_turns_left;
    
} MpuReading_t;

class Websocket {
private:
	esp_websocket_client_handle_t client;
	esp_websocket_client_config_t websocket_cfg;


	
public:

 	Websocket();

	void ConfigureWebsocket(char *uri);
	void SendRawData(int gyroX, int gyroY, int gyroZ, int accelX, int accelY, int accelZ, float encoderR, float encoderL);
	void SendBatchData(const std::vector<MpuReading_t>& batch);


};
#endif
#endif 