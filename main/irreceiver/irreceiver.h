// irreceiver.h
#ifndef IRRECEIVER_H
#define IRRECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class IRReceiver {
private:
    gpio_num_t _gpio_pin;
    rmt_channel_handle_t _rx_channel = NULL;
	rmt_channel_handle_t _tx_channel = NULL;
	rmt_rx_done_event_data_t _rx_data;
	rmt_symbol_word_t raw_symbols[64];
	uint8_t _protocol;
	uint32_t _signal_range_min_ns, _signal_range_max_ns;
	uint32_t _signal_low_min_limit, _signal_low_max_limit;
	uint32_t _signal_high_min_limit, _signal_high_max_limit;
	uint8_t _command_init, _command_finish;
	
	QueueHandle_t _receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
	rmt_receive_config_t receive_config;
public:
	IRReceiver(gpio_num_t, uint8_t);
	~IRReceiver() {}
    void ConfigureIRReceiver();
    uint8_t ReadReceiver();
    uint8_t parse_signal_frame(rmt_symbol_word_t *rmt_symbols, size_t symbol_num);

};
#endif
#endif // DSHOT_H