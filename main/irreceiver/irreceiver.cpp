#include "irreceiver.h"
#include <cstdlib>

#define EXAMPLE_IR_RESOLUTION_HZ     1000000

typedef enum {
	NEC = 0,
	SIRC
} protocols;

static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

// Construtor da classe Dshot
IRReceiver::IRReceiver(gpio_num_t gpio_pin, uint8_t protocol) {
    _gpio_pin = gpio_pin;
    _protocol = protocol;
    
    if (protocol == NEC) {
		_signal_range_min_ns = 1250;
		_signal_range_max_ns = 12000000;
		_signal_low_min_limit = 1050;
		_signal_low_max_limit = 1250;
		_signal_high_min_limit = 2100;
		_signal_high_max_limit = 2400;
		_command_init = 17,
		_command_finish = 25;
	}
	if (protocol == SIRC) {
		_signal_range_min_ns = 1250;
		_signal_range_max_ns = 12000000;
		_signal_low_min_limit = 1050;
		_signal_low_max_limit = 1350;
		_signal_high_min_limit = 1650;
		_signal_high_max_limit = 1950;
		_command_init = 1,
		_command_finish = 7;
	}
	
}

void IRReceiver::ConfigureIRReceiver() {
	rmt_rx_channel_config_t rx_channel_cfg = {
		.gpio_num = _gpio_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
        .mem_block_symbols = 64 // amount of RMT symbols that the channel can store at a time
    };
    rmt_new_rx_channel(&rx_channel_cfg, &_rx_channel);
    
 
    assert(_receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    rmt_rx_register_event_callbacks(_rx_channel, &cbs, _receive_queue);
    receive_config = {
        .signal_range_min_ns = _signal_range_min_ns,     // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
        .signal_range_max_ns = _signal_range_max_ns, // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
    };
    
    rmt_enable(_rx_channel);
    rmt_receive(_rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config);
}

uint8_t IRReceiver::parse_signal_frame(rmt_symbol_word_t *rmt_symbols, size_t symbol_num)
{
	uint8_t signal = 0;
	uint32_t symbol_sum;
	if (_protocol == NEC) {
		printf("NEC frame start---\r\n");
	    for (size_t i = 0; i < symbol_num; i++) {
	        printf("%d - {%d:%d},{%d:%d}\r\n", i, rmt_symbols[i].level0, rmt_symbols[i].duration0,
	               rmt_symbols[i].level1, rmt_symbols[i].duration1);
           	
	    }
	    for (uint8_t i = _command_init; i < _command_finish; i++) {
			symbol_sum = rmt_symbols[i].duration0 + rmt_symbols[i].duration1;
			if (symbol_sum > _signal_low_min_limit && symbol_sum < _signal_low_max_limit) {
				signal = signal >> 1;
			} else if (symbol_sum > _signal_high_min_limit && symbol_sum < _signal_high_max_limit) {
				signal = (signal >> 1) | 128;
			}
		}
	    printf("\nSignal result %u\n", signal);
    	printf("---NEC frame end: ");
    	return signal;
	}
	if (_protocol == SIRC) {
		printf("SIRC frame start---\r\n");
	    for (size_t i = 0; i < symbol_num; i++) {
	        printf("%d - {%d:%d},{%d:%d}\r\n", i, rmt_symbols[i].level0, rmt_symbols[i].duration0,
	               rmt_symbols[i].level1, rmt_symbols[i].duration1);
           	
	    }
	    for (uint8_t i = _command_init; i < _command_finish; i++) {
			symbol_sum = rmt_symbols[i].duration0 + rmt_symbols[i].duration1;
			if (symbol_sum > _signal_low_min_limit && symbol_sum < _signal_low_max_limit) {
				signal = signal >> 1;
			} else if (symbol_sum > _signal_high_min_limit && symbol_sum < _signal_high_max_limit) {
				signal = (signal >> 1) | 128;
			}
		}
		signal = signal >> 1;
	    printf("\nSignal result %u\n", signal);
    	printf("---SIRC frame end: ");
    	return signal;
	}	
	return 0;
}

uint8_t IRReceiver::ReadReceiver() {
	uint8_t result = 0;
	if (xQueueReceive(_receive_queue, &_rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
		result = parse_signal_frame(_rx_data.received_symbols, _rx_data.num_symbols);
		rmt_receive(_rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config);
	}
	return result;
}


