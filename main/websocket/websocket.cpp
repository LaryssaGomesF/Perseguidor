#include "websocket.h"

#include "esp_timer.h"


Websocket::Websocket() {}

static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI("Websocket", "WebSocket conectado");
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI("Websocket", "WebSocket desconectado");
            break;
        case WEBSOCKET_EVENT_DATA:
            ESP_LOGI("Websocket", "Recebido: %.*s", data->data_len, (char *)data->data_ptr);
            break;
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE("Websocket", "Erro no WebSocket");
            break;
    }
}

void Websocket::ConfigureWebsocket(char *uri) {
	websocket_cfg = {
        .uri = uri,
    };
    client = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);
    esp_websocket_client_start(client);
 
}

void Websocket::SendRawData(int gyroX, int gyroY, int gyroZ, int accelX, int accelY, int accelZ, int encoderR, int encoderL){
	 if (esp_websocket_client_is_connected(client)) {
            char msg[180];        
			int64_t timestamp_ms = esp_timer_get_time() / 1000; 
            snprintf(msg, sizeof(msg), "{\"gyroX\": %d, \"gyroY\": %d, \"gyroZ\": %d, \"accelX\": %d, \"accelY\": %d, \"accelZ\": %d,\"encoderR\": %d,\"encoderL\": %d, \"timestamp\": %lld}", gyroX, gyroY, gyroZ, accelX, accelY, accelZ, encoderR, encoderL, timestamp_ms);
            esp_websocket_client_send_text(client, msg, strlen(msg), portMAX_DELAY);
            ESP_LOGI("Websocket", "Enviado: %s", msg);
        }
}


void Websocket::SendData(float roll, float pitch, float yaw){
	 if (esp_websocket_client_is_connected(client)) {
            char msg[180];        
			int64_t timestamp_ms = esp_timer_get_time() / 1000; 
            snprintf(msg, sizeof(msg), "{\"roll\":%.2f, \"pitch\": %.2f, \"yaw\": %.2f, \"timestamp\": %lld}", roll, pitch, yaw, timestamp_ms);
            esp_websocket_client_send_text(client, msg, strlen(msg), portMAX_DELAY);
            ESP_LOGI("Websocket", "Enviado: %s", msg);
        }
}

