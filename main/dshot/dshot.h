// dshot.h
#ifndef DSHOT_H
#define DSHOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"
#include <sys/_stdint.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Dshot {
private:
    gpio_num_t _gpio_pin;
    rmt_channel_handle_t _esc_chan = NULL;
    rmt_encoder_handle_t _dshot_encoder = NULL;
    dshot_esc_throttle_t _throttle;
    rmt_transmit_config_t _tx_config;
    bool _direction;
    uint16_t _limit;

public:
	Dshot(gpio_num_t, bool, uint16_t);
	~Dshot() {}
    void ConfigureDshot();
    void UpdateThrottle(int);
};
#endif
#endif // DSHOT_H