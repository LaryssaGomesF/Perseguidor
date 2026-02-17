#include "mqtt_config.h"
#include "esp_log.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "inttypes.h"
static const char *TAG = "MQTT_SIMPLE";

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Conectado ao Broker!");
            //send_json_data(client); // Envia assim que conectar
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "Publicação confirmada (QoS 1), msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Erro no MQTT");
            break;
        default:
            break;
    }
}

/*void Mqtt_Config::ConfigureMqtt() {
	esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = "mqtt://192.168.1.25";
    mqtt_cfg.broker.address.port = 1883;

    _client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(_client);
}*/

void Mqtt_Config::ConfigureMqtt() {
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = "mqtt://192.168.1.25";
    mqtt_cfg.broker.address.port = 1883;

    // Aumente o buffer para suportar mensagens maiores e o acúmulo de pacotes aguardando ACK
    mqtt_cfg.buffer.size = 2048;      // Buffer de recepção/transmissão
    mqtt_cfg.buffer.out_size = 2048;  // Buffer específico de saída

    // Opcional: Aumentar o limite de mensagens pendentes (outbox)
    // Isso evita que a função publish retorne erro imediatamente se a rede oscilar
    mqtt_cfg.session.protocol_ver = MQTT_PROTOCOL_V_3_1_1; // Auto detect

    _client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(_client);
}

void Mqtt_Config::publishData(int16_t gyroX, int16_t gyroY, int16_t gyroZ, int16_t accelX, int16_t accelY, int16_t accelZ, float total_accumulated_degrees_right, float total_accumulated_degrees_left, uint32_t ts){
	char payload[1024];

    snprintf(payload, sizeof(payload),
             "[%" PRIu32 ",%d,%d,%d,%d,%d,%d,%.2f,%.2f]",
             ts,
             gyroX, gyroY, gyroZ,
             accelX, accelY, accelZ,
             total_accumulated_degrees_right,
             total_accumulated_degrees_left);

    esp_mqtt_client_publish(
        _client,
        "robot/sensors",
        payload,
        0,
        1,
        0
    );
}

void Mqtt_Config::publishRaw(const char* payload) {
    if (_client == NULL) return;


    int msg_id = esp_mqtt_client_publish(
        _client,
        "robot/sensors",
        payload,
        0,   // Tamanho (0 = usar strlen do payload)
        2,   // QoS 0 (Importante: não espera confirmação, evita gaps)
        0    // Retain off
    );

    if (msg_id < 0) {
        // Opcional: Log de erro se o buffer de saída do MQTT estiver cheio
        // ESP_LOGE("MQTT", "Falha ao publicar, buffer cheio");
    }
}