#include "websocket.h"



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

void Websocket::SendData(){
	 if (esp_websocket_client_is_connected(client)) {
            char msg[64];
            float gyro_x = 1.23; // Simulado
            int encoder_ticks = 456; // Simulado

            snprintf(msg, sizeof(msg), "{\"gyro\": %.2f, \"encoder\": %d}", gyro_x, encoder_ticks);
            esp_websocket_client_send_text(client, msg, strlen(msg), portMAX_DELAY);
            ESP_LOGI("Websocket", "Enviado: %s", msg);
        }
}
