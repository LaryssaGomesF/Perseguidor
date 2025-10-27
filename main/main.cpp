#include <iostream>
#include <numbers>
#include <stdio.h>
#include <stdbool.h>
#include <sys/_intsup.h>
#include <sys/_stdint.h>
#include <unistd.h>
#include "arch/sys_arch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "dshot/dshot.h"
#include "irreceiver/irreceiver.h"
#include "soc/adc_channel.h"
#include "soc/gpio_num.h"
#include "gpio/gpio_config.h"
#include "esp_adc/adc_continuous.h"
#include "esp_bit_defs.h"
#include "hal/adc_types.h"
#include "adc/adc.h"
#include "adccontinuos/adccontinuos.h"
#include "encoder/encoder.h"
#include "mpu/mpu.h"
#include "wifi/wifi.h"
#include "websocket/websocket.h"
#include "as5600/as5600.h"
#include "driver/mcpwm_cap.h"

//MOTORES
#define DSHOT_MOTOR_R 15
#define DSHOT_MOTOR_L 13
#define MOTOR_MAX_SPEED 90
#define MOTOR_BASE_SPEED 50

//LED
#define GPIO_MOSFET 23
#define GPIO_LED1 17
#define GPIO_LED2 5

//IR RECEIVE
#define IR_RECEIVER_PIN 2

//MPU
#define GPIO_MPU_SDA 22
#define GPIO_MPU_SCL 21
#define I2C_PORT I2C_NUM_0

//ENCODER
#define AS5600_RIGHT_PWM_INPUT_GPIO 18
#define AS5600_LEFT_PWM_INPUT_GPIO 4
#define MCPWM_TIMER_CLK_HZ 80000000  // 80 MHz clock base

//WIFI
#define WIFI_SSID "RAYANE"
#define WIFI_PASS "35523839"
#define WEBSOCKET_URI "ws://192.168.1.25:8765/"

// 0 - White line on a black surface
// 1 - Black line on a white surface
#define TRACE_COLOR	0




// Handles for the freeRTOS tasks
TaskHandle_t xHandleCalibration = NULL;
TaskHandle_t xHandleAdcContinuos = NULL;
TaskHandle_t xHandleFollowLine = NULL;
TaskHandle_t xHandleReadLine = NULL;
TaskHandle_t xHandleReadSensor = NULL;
TaskHandle_t xHandleCountCheckpoint = NULL;
TaskHandle_t xHandleIRMonitor = NULL;
TaskHandle_t xHandleReadMpu = NULL;
TaskHandle_t xHandleSendData = NULL;

TaskHandle_t  xHandleTaskCalculeAngleEncoderLeft = NULL;
TaskHandle_t  xHandleTaskCalculeAngleEncoderRight = NULL;

			 
// Classes presenting the project components
Gpio mosfet(static_cast<gpio_num_t>(GPIO_MOSFET));
Gpio ledWhite(static_cast<gpio_num_t>(GPIO_LED1));
Gpio ledYellow(static_cast<gpio_num_t>(GPIO_LED2));

Dshot motorL(static_cast<gpio_num_t>(DSHOT_MOTOR_L), false, MOTOR_MAX_SPEED);
Dshot motorR(static_cast<gpio_num_t>(DSHOT_MOTOR_R), true, MOTOR_MAX_SPEED);

adc_channel_t adc1[6] = {ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_0, ADC_CHANNEL_5};
adc_channel_t adc2[3] = {ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_9};
Adc adc( adc2, TRACE_COLOR);
AdcContinuos adcContinuos(adc1,  TRACE_COLOR );


As5600 encoderR(static_cast<gpio_num_t>(AS5600_RIGHT_PWM_INPUT_GPIO));
As5600 encoderL(static_cast<gpio_num_t>(AS5600_LEFT_PWM_INPUT_GPIO));


Mpu mpu(static_cast<gpio_num_t>(GPIO_MPU_SDA),static_cast<gpio_num_t>(GPIO_MPU_SCL) , static_cast<i2c_port_num_t>(I2C_PORT));


// for NEC protocol
const int start_key = 69, stop_key = 70;
bool protocol = 0;


IRReceiver IRSensor(static_cast<gpio_num_t>(IR_RECEIVER_PIN), protocol);

Wifi wifi;
Websocket websocket;


float adcSensors[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float adcSideSensors[3] = {0.0, 0.0, 0.0};


const float kp = 1.10, kd = 0.0, max_accel = 1;

uint8_t command;

int gyroX, gyroY, gyroZ, accelX, accelY, accelZ = 0;

// Contagem de marcações laterais
static int countR = 0, countL= 0;

int rotation_count = 0;
float last_encoder_angle = -1.0f;

const float PULSE_WIDTH_MIN_US = 32.0f; 

// A faixa de largura de pulso que efetivamente contém os dados do ângulo (4095 ticks do sensor @ 920Hz)
const float PULSE_WIDTH_DATA_RANGE_US = 1024.0f;

static volatile uint32_t last_pos_edge_timestamp_left = 0;
static volatile uint32_t last_period_ticks_left = 0;
static volatile uint32_t last_high_ticks_left = 0;

static volatile uint32_t last_pos_edge_timestamp_right = 0;
static volatile uint32_t last_period_ticks_right = 0;
static volatile uint32_t last_high_ticks_right = 0;

const int CALIBRATION_SAMPLES_MPU = 2000; 
int countCalibrationMPU = 0;

const int CALIBRATION_SAMPLES = 20000; 
int countCalibration = 0;

float last_degre_angle_left = 0;
float last_degre_angle_right = 0;

bool start = false;

void readMpu(void *parameters) {

    for (;;) {		
		mpu.ReadMPU(&gyroX, &gyroY, &gyroZ, &accelX, &accelY, &accelZ);
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}


void sendData(void *parameters) {

    for (;;) {
		if (start){			
	        websocket.SendRawData(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, last_degre_angle_left, last_degre_angle_right);
		}
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}



void countCheckpoint(void *parameters){
	static bool stateL = false, stateR = false;
	
	printf("countCheckpoint");
	
	for (;;) {
		
		//Left 7
		adcSideSensors[0] = adc.ReadAdc(0);
		adcSideSensors[1] = adc.ReadAdc(1);	
		adcSideSensors[2] = adc.ReadAdc(2);		
		/*if(adcSideSensors[0] > 2.5 && stateL == false){
			
			stateL = true;
			countL++;
		} 
		if(adcSideSensors[0] < 2.5 && stateL == true){
			stateL = false;
			
		}
		
		//Right 8
		
		if(adcSideSensors[1] > 2.5 && stateR == false){
			ledWhite.UpdateGPIO(true);
			ledYellow.UpdateGPIO(true);
			stateR = true;
			countR++;
		} 
		if(adcSideSensors[1] < 2.5 && stateR == true){
			ledWhite.UpdateGPIO(false);
			ledYellow.UpdateGPIO(false);
			stateR = false;
		}*/
		printf("s1: %.2f, s2: %.2f, s3: %.2f\n", adcSideSensors[0],	adcSideSensors[1], adcSideSensors[2]);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void readLine(void *parameters){
	for(;;){
		adcContinuos.ReadAdc(&adcSensors);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}
	
}


void followLine(void *parameters) {
	static float error = 0, lastError = 0, sensorSum = 0 ;

	for(;;) {  

		
		if (start) {
		lastError = error;
		sensorSum = adcSensors[0] + adcSensors[1] + adcSensors[2] + adcSensors[3]+ adcSensors[4] + adcSensors[5];		
	 	
		if(sensorSum < 5) {
			motorL.UpdateThrottle(0);
	   		motorR.UpdateThrottle(0);
			
		} else {
			error = (adcSensors[1] - adcSensors[0]) + 1.4*(adcSensors[3] - adcSensors[2]) + 1.8*(adcSensors[5] - adcSensors[4]);
	    	float controllerResult = kp*error + kd*(error - lastError);
			motorR.UpdateThrottle(MOTOR_BASE_SPEED + controllerResult );
	   		motorL.UpdateThrottle(MOTOR_BASE_SPEED - controllerResult );
		}
		
		
	  } else {
		motorR.UpdateThrottle(0);	
	    motorL.UpdateThrottle(0);
	  }
	  vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void irmonitor(void *parameters) {
	for(;;) {
		command = IRSensor.ReadReceiver();
		if (command == start_key) start = true;
		if (command == stop_key) start = false;
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}




static bool IRAM_ATTR callbackAdcContinuos(adc_continuous_ctx_t *handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t mustYield1 = pdFALSE;
    BaseType_t mustYield2 = pdFALSE;

    if ((xHandleReadLine != NULL) && (eTaskGetState(xHandleReadLine) != eSuspended)) {
        vTaskNotifyGiveFromISR(xHandleReadLine, &mustYield1);
       
    }

    if ((xHandleCalibration != NULL) && (eTaskGetState(xHandleCalibration) != eSuspended)) {
        vTaskNotifyGiveFromISR(xHandleCalibration, &mustYield2);
    }

    return (mustYield1 || mustYield2);  // Retorna true se alguma task precisar de troca de contexto
}



static bool pwm_capture_channel_callback_left(mcpwm_cap_channel_handle_t cap_chan,
                                         const mcpwm_capture_event_data_t *edata,
                                         void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // Nova borda de subida → calcula período desde a última borda de subida
        if (last_pos_edge_timestamp_left != 0) {
            last_period_ticks_left = edata->cap_value - last_pos_edge_timestamp_left;
        }
        last_pos_edge_timestamp_left = edata->cap_value;
    } else { // Borda de descida
        if (last_pos_edge_timestamp_left != 0) {
            last_high_ticks_left = edata->cap_value - last_pos_edge_timestamp_left;
            // Notifica a tarefa principal que há novos dados disponíveis
            xTaskNotifyFromISR(xHandleTaskCalculeAngleEncoderLeft, 1, eSetValueWithOverwrite, &high_task_wakeup);
        }
    }

    return high_task_wakeup == pdTRUE;
}

static bool pwm_capture_channel_callback_right(mcpwm_cap_channel_handle_t cap_chan,
                                         const mcpwm_capture_event_data_t *edata,
                                         void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // Nova borda de subida → calcula período desde a última borda de subida
        if (last_pos_edge_timestamp_right != 0) {
            last_period_ticks_right = edata->cap_value - last_pos_edge_timestamp_right;
        }
        last_pos_edge_timestamp_right = edata->cap_value;
    } else { // Borda de descida
        if (last_pos_edge_timestamp_right != 0) {
            last_high_ticks_right = edata->cap_value - last_pos_edge_timestamp_right;
            // Notifica a tarefa principal que há novos dados disponíveis
            xTaskNotifyFromISR(xHandleTaskCalculeAngleEncoderRight, 1, eSetValueWithOverwrite, &high_task_wakeup);
        }
    }

    return high_task_wakeup == pdTRUE;
}

void countEncoderLeft(void *parameters) {
    ESP_LOGI("ENCODER_TASK", "Tarefa de leitura iniciada, aguardando notificacoes...");
	for(;;) {
        // --- ESPERA PELA NOTIFICAÇÃO ---
        // A tarefa fica bloqueada aqui, sem consumir CPU, por até 1000ms.
        // Se a ISR chamar xTaskNotifyFromISR, a tarefa acorda imediatamente.
        if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE){
            
            // Copia o valor volátil para uma variável local
            uint32_t high_ticks = last_high_ticks_left;

            // Converte para microssegundos
            float t_high_us = (float)high_ticks / (MCPWM_TIMER_CLK_HZ / 1e6f);
            
            // Lógica de cálculo do ângulo
            float data_pulse_width_us = t_high_us - PULSE_WIDTH_MIN_US;

            if (data_pulse_width_us < 0) data_pulse_width_us = 0;
            if (data_pulse_width_us > PULSE_WIDTH_DATA_RANGE_US) data_pulse_width_us = PULSE_WIDTH_DATA_RANGE_US;

            last_degre_angle_left = (data_pulse_width_us / PULSE_WIDTH_DATA_RANGE_US) * 360.0f;

        } 
          
	}
}

void countEncoderRight(void *parameters) {
    ESP_LOGI("ENCODER_TASK", "Tarefa de leitura iniciada, aguardando notificacoes...");
	for(;;) {
        // --- ESPERA PELA NOTIFICAÇÃO ---
        // A tarefa fica bloqueada aqui, sem consumir CPU, por até 1000ms.
        // Se a ISR chamar xTaskNotifyFromISR, a tarefa acorda imediatamente.
        if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE){
            
            // Copia o valor volátil para uma variável local
            uint32_t high_ticks = last_high_ticks_right;

            // Converte para microssegundos
            float t_high_us = (float)high_ticks / (MCPWM_TIMER_CLK_HZ / 1e6f);
            
            // Lógica de cálculo do ângulo
            float data_pulse_width_us = t_high_us - PULSE_WIDTH_MIN_US;

            if (data_pulse_width_us < 0) data_pulse_width_us = 0;
            if (data_pulse_width_us > PULSE_WIDTH_DATA_RANGE_US) data_pulse_width_us = PULSE_WIDTH_DATA_RANGE_US;

            last_degre_angle_right = (data_pulse_width_us / PULSE_WIDTH_DATA_RANGE_US) * 360.0f;


        } 
	}
}

void settingEncoders(){
	mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_timer_conf = {
        .group_id = 0,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_timer_conf, &cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

	
	xTaskCreatePinnedToCore(countEncoderLeft, "Encoder Counter Task", 4096, NULL, 1, &xHandleTaskCalculeAngleEncoderLeft, 1);
	xTaskCreatePinnedToCore(countEncoderRight, "Encoder Counter Task", 4096, NULL,1, &xHandleTaskCalculeAngleEncoderRight, 1);
	xTaskCreatePinnedToCore(readMpu, "ReadMPU", 4096, NULL, 1, &xHandleReadMpu, 1);

	xTaskCreatePinnedToCore(sendData, "sendData", 4096, NULL, 2, &xHandleSendData, 1);

 	encoderR.ConfigureAS5600(cap_timer, pwm_capture_channel_callback_right);
    encoderL.ConfigureAS5600(cap_timer, pwm_capture_channel_callback_left);
}

void calibration(void *parameters) {
	for(;;) {
		
		mosfet.UpdateGPIO(true);
		ledWhite.UpdateGPIO(true);
		int speed = MOTOR_BASE_SPEED + 15;		
		motorR.UpdateThrottle(speed);
	   	motorL.UpdateThrottle(-speed);
	   	adcContinuos.Calibration();	
		countCalibration++;
		/*adc.Calibration(0);
		adc.Calibration(1);
		adc.Calibration(2);*/
		
		if(countCalibration > CALIBRATION_SAMPLES){
			
			printf("stop calibracao\n\n\n");
			motorR.UpdateThrottle(0);
			motorL.UpdateThrottle(0);
			ledWhite.UpdateGPIO(false);		
			adcContinuos.GetMinAndMaxValues();
			//adc.GetMinAndMaxValues();
			
			settingEncoders();
			xTaskCreatePinnedToCore(followLine, "FollowLine", 4096, NULL, 1, &xHandleFollowLine, 0);
			xTaskCreatePinnedToCore(readLine, "ReadLine", 4096, NULL, 1, &xHandleReadLine, 0);
			xTaskCreatePinnedToCore(irmonitor, "IRMonitor", 4096, NULL, 1, &xHandleIRMonitor, 1);
			//xTaskCreatePinnedToCore(countCheckpoint, "CountCheckpoints", 4096, NULL, 2, &xHandleCountCheckpoint, 1);
			
			vTaskSuspend(xHandleCalibration);
			vTaskDelete(NULL);
		}
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		
	}
}

extern "C" void app_main(void)
{
	printf("Running Experiment262");
	
	//LED
	ledWhite.ConfigureGPIO();
	ledYellow.ConfigureGPIO();	
	
	//MOTORES
	motorL.ConfigureDshot();	
	motorR.ConfigureDshot();	
	mosfet.ConfigureGPIO();
	mosfet.UpdateGPIO(true);
	
	
    //WIFI
	ESP_ERROR_CHECK(nvs_flash_init());
	wifi.ConfigureWifi();
	wifi.ConnectWifi(WIFI_SSID, WIFI_PASS);	
	vTaskDelay(pdMS_TO_TICKS(3000));
	
	
	//WEBSOCKET	
	websocket.ConfigureWebsocket(WEBSOCKET_URI);
	
	
	//ADC
	//adc.ConfigureAdc();
	adcContinuos.ConfigureAdc(callbackAdcContinuos);
	vTaskDelay(pdMS_TO_TICKS(3000));
	
    // IR RECEIVER
	IRSensor.ConfigureIRReceiver();
	
	
	// MPU E ENCODERS
	mpu.ConfigureMPU();

	xTaskCreatePinnedToCore(calibration, "Calibration", 4096, NULL, 24, &xHandleCalibration, 0);
	vTaskDelay(pdMS_TO_TICKS(3000));

}
