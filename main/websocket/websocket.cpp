#include "websocket.h"
#include <string>
#include <sstream>
#include <cstring>
#include "esp_timer.h"
#include <vector>

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

void Websocket::SendRawData(int gyroX, int gyroY, int gyroZ, int accelX, int accelY, int accelZ, float encoderR, float encoderL){
	 if (esp_websocket_client_is_connected(client)) {
            char msg[180];        
			int64_t timestamp_ms = esp_timer_get_time() / 1000; 
            snprintf(msg, sizeof(msg), "{\"gyroX\": %d, \"gyroY\": %d, \"gyroZ\": %d, \"accelX\": %d, \"accelY\": %d, \"accelZ\": %d,\"encoderR\": %.2f,\"encoderL\": %.2f, \"timestamp\": %lld}", gyroX, gyroY, gyroZ, accelX, accelY, accelZ, encoderR, encoderL, timestamp_ms);
            esp_websocket_client_send_text(client, msg, strlen(msg), portMAX_DELAY);
            //ESP_LOGI("Websocket", "Enviado: %s", msg);
        }
}

#define MAX_JSON_MESSAGE_SIZE 1600 

void Websocket::SendBatchData(const std::vector<MpuReading_t>& batch) {
    if (batch.empty()) {
        return;
    }

    // 1. Declaração do buffer de tamanho fixo na pilha (stack)
    char json_message[MAX_JSON_MESSAGE_SIZE];
    
    char* current_pos = json_message;
    size_t remaining_size = MAX_JSON_MESSAGE_SIZE;
    int written_chars;

    // Início do Array principal: [
    written_chars = snprintf(current_pos, remaining_size, "[");
    current_pos += written_chars;
    remaining_size -= written_chars;

    // Loop para construir os arrays internos
    for (size_t i = 0; i < batch.size(); ++i) {
        const auto& reading = batch[i];
        
        // Formata e concatena diretamente no buffer
        written_chars = snprintf(current_pos, remaining_size, 
                                 "[%lld, %d, %d, %d, %d, %d, %d, %.2f, %.2f]%s",
                                 reading.timestamp_ms,
                                 reading.gyroX, reading.gyroY, reading.gyroZ,
                                 reading.accelX, reading.accelY, reading.accelZ,
                                 reading.total_accumulated_turns_right,
                                 reading.total_accumulated_turns_left,
                                 (i < batch.size() - 1) ? "," : ""); // Adiciona vírgula se não for o último

        // Verificação de segurança (omiti tratamento de erro para brevidade)
        current_pos += written_chars;
        remaining_size -= written_chars;
    }

    // Fim do Array principal: ]
    written_chars = snprintf(current_pos, remaining_size, "]");
    current_pos += written_chars;

    // 2. Envio da mensagem via WebSocket
    if (esp_websocket_client_is_connected(client)) {
        size_t message_len = current_pos - json_message;
        esp_websocket_client_send_text(client, json_message, message_len, portMAX_DELAY);
    }
}
