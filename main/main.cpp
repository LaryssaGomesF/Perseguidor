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
#define DSHOT_MOTOR_R 13
#define DSHOT_MOTOR_L 14
#define MOTOR_MAX_SPEED 90
#define MOTOR_BASE_SPEED 50

//LED
#define GPIO_MOSFET 23
#define GPIO_LED1 17
#define GPIO_LED2 5

//IR RECEIVE
#define IR_RECEIVER_PIN 15

//MPU
#define GPIO_MPU_SDA 22
#define GPIO_MPU_SCL 21
#define I2C_PORT I2C_NUM_0

//ENCODER
#define AS5600_SDA_PIN  15
#define AS5600_SCL_PIN  2
#define AS5600_I2C_PORT I2C_NUM_1 
#define AS5600_PWM_INPUT_GPIO 13
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
TaskHandle_t xHandleReadMpuEncoder = NULL;

TaskHandle_t  xHandleTaskCalculeAngleEncoder = NULL;

			 
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


As5600 encoderL(static_cast<gpio_num_t>(AS5600_SDA_PIN), static_cast<gpio_num_t>(AS5600_SCL_PIN),static_cast<i2c_port_num_t>(AS5600_I2C_PORT), static_cast<gpio_num_t>(AS5600_PWM_INPUT_GPIO));

Mpu mpu(static_cast<gpio_num_t>(GPIO_MPU_SDA),static_cast<gpio_num_t>(GPIO_MPU_SCL) , static_cast<i2c_port_num_t>(I2C_PORT));


// for NEC protocol
const int start_key = 69, stop_key = 70;
bool protocol = 0;


IRReceiver IRSensor(static_cast<gpio_num_t>(IR_RECEIVER_PIN), protocol);

Wifi wifi;
Websocket websocket;


float adcSensors[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float adcSideSensors[2] = {0.0, 0.0};


const float kp = 1.10, kd = 0.0, max_accel = 1;

uint8_t command;

// Contagem de marcações laterais
static int countR = 0, countL= 0;

int rotation_count = 0;
float last_encoder_angle = -1.0f;

const float PULSE_WIDTH_MIN_US = 32.0f; 

// A faixa de largura de pulso que efetivamente contém os dados do ângulo (4095 ticks do sensor @ 920Hz)
const float PULSE_WIDTH_DATA_RANGE_US = 1024.0f;

static volatile uint32_t last_pos_edge_timestamp = 0;
static volatile uint32_t last_period_ticks = 0;
static volatile uint32_t last_high_ticks = 0;



const int CALIBRATION_SAMPLES_MPU = 200; 
int countCalibrationMPU = 0;

const int CALIBRATION_SAMPLES = 20000; 
int countCalibration = 0;



void readMpuEncoder(void *parameters) {
    // Adicione a variável 'yaw'
    float roll, pitch, yaw;

    uint32_t last_update_time = esp_log_timestamp();
    float dt;
    
    /*while (last_encoder_angle < 0) {
        processEncoderData(); // Chama a nova função para obter a primeira leitura
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGI("SENSOR_TASK", "Primeira leitura do encoder: %.2f graus. Iniciando loop principal.", last_encoder_angle);*/


    for (;;) {
        // Calcula o dt
        uint32_t current_time = esp_log_timestamp();
        dt = (current_time - last_update_time) / 1000.0f;
        last_update_time = current_time;

        // --- CORREÇÃO AQUI ---
        // Modifique a chamada para obter os 3 ângulos
        //mpu.GetFilteredAngles(&roll, &pitch, &yaw, dt);
		//mpu.ReadMPU(int *gyroX, int *gyroY, int *gyroZ, int *accelX, int *accelY, int *accelZ)
		// Ler encoder
		//float current_encoder_angle = encoderL.ReadRawAngle();
		//printf("Encoder: %.2f\n",current_encoder_angle);
        // Envie os 3 ângulos reais para o servidor
        //websocket.SendData(roll, pitch, yaw);

        // Imprime os 3 ângulos para depuração
        //printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}




void countCheckpoint(void *parameters){
	static bool stateL = false, stateR = false;
	
	printf("countCheckpoint");
	
	for (;;) {
		
		//Left 7
		adcSideSensors[0] = adc.ReadAdc(0);
		if(adcSideSensors[0] > 2.5 && stateL == false){
			
			stateL = true;
			countL++;
		} 
		if(adcSideSensors[0] < 2.5 && stateL == true){
			stateL = false;
			
		}
		
		//Right 8
		adcSideSensors[1] = adc.ReadAdc(1);
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
		}
		vTaskDelay(pdMS_TO_TICKS(10));
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

	static bool start = false;
	
	for(;;) {  
		if (command == start_key) start = true;
		
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
		
		if (command == stop_key) start = false;
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




void calibration(void *parameters) {
	for(;;) {
		
		mosfet.UpdateGPIO(true);
		ledWhite.UpdateGPIO(true);
		int speed = MOTOR_BASE_SPEED + 15;		
		motorR.UpdateThrottle(speed);
	   	motorL.UpdateThrottle(-speed);
	   	adcContinuos.Calibration();	
		countCalibration++;
		//adc.Calibration(0);
		//adc.Calibration(1);
		//adc.Calibration(2);
		if(countCalibration > CALIBRATION_SAMPLES){
			
			printf("stop calibracao\n\n\n");
			motorR.UpdateThrottle(0);
			motorL.UpdateThrottle(0);
			ledWhite.UpdateGPIO(false);		
			adcContinuos.GetMinAndMaxValues();
			//adc.GetMinAndMaxValues();
			
			
			xTaskCreatePinnedToCore(followLine, "FollowLine", 4096, NULL, 2, &xHandleFollowLine, 0);
			xTaskCreatePinnedToCore(readLine, "ReadLine", 4096, NULL, 2, &xHandleReadLine, 0);
			
		

			xTaskCreatePinnedToCore(irmonitor, "IRMonitor", 4096, NULL, 2, &xHandleIRMonitor, 1);
			xTaskCreatePinnedToCore(readMpuEncoder, "ReadMPU", 4096, NULL, 2, &xHandleReadMpuEncoder, 1);
			xTaskCreatePinnedToCore(countCheckpoint, "CountCheckpoints", 4096, NULL, 2, &xHandleCountCheckpoint, 1);
			
			vTaskSuspend(xHandleCalibration);
			vTaskDelete(NULL);
		}
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}
}



static bool pwm_capture_channel_callback(mcpwm_cap_channel_handle_t cap_chan,
                                         const mcpwm_capture_event_data_t *edata,
                                         void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // Nova borda de subida → calcula período desde a última borda de subida
        if (last_pos_edge_timestamp != 0) {
            last_period_ticks = edata->cap_value - last_pos_edge_timestamp;
        }
        last_pos_edge_timestamp = edata->cap_value;
    } else { // Borda de descida
        if (last_pos_edge_timestamp != 0) {
            last_high_ticks = edata->cap_value - last_pos_edge_timestamp;
            // Notifica a tarefa principal que há novos dados disponíveis
            xTaskNotifyFromISR(xHandleTaskCalculeAngleEncoder, 1, eSetValueWithOverwrite, &high_task_wakeup);
        }
    }

    return high_task_wakeup == pdTRUE;
}

void countencoder(void *parameters) {
    ESP_LOGI("ENCODER_TASK", "Tarefa de leitura iniciada, aguardando notificacoes...");
	for(;;) {
        // --- ESPERA PELA NOTIFICAÇÃO ---
        // A tarefa fica bloqueada aqui, sem consumir CPU, por até 1000ms.
        // Se a ISR chamar xTaskNotifyFromISR, a tarefa acorda imediatamente.
        if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(1000)) == pdTRUE) {
            
            // Copia o valor volátil para uma variável local
            uint32_t high_ticks = last_high_ticks;

            // Converte para microssegundos
            float t_high_us = (float)high_ticks / (MCPWM_TIMER_CLK_HZ / 1e6f);
            
            // Lógica de cálculo do ângulo
            float data_pulse_width_us = t_high_us - PULSE_WIDTH_MIN_US;

            if (data_pulse_width_us < 0) data_pulse_width_us = 0;
            if (data_pulse_width_us > PULSE_WIDTH_DATA_RANGE_US) data_pulse_width_us = PULSE_WIDTH_DATA_RANGE_US;

            float angle_deg = (data_pulse_width_us / PULSE_WIDTH_DATA_RANGE_US) * 360.0f;

            ESP_LOGI("ENCODER_TASK", "t_high = %.2fus | angle = %.2f°", t_high_us, angle_deg);

        } else {
            // Este bloco só é executado se a notificação não chegar em 1000ms.
            ESP_LOGW("ENCODER_TASK", "Timeout: Nenhum pulso PWM detectado.");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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
	vTaskDelay(pdMS_TO_TICKS(5000));
	
	//ADC
	adc.ConfigureAdc();
	adcContinuos.ConfigureAdc(callbackAdcContinuos);
	
	//WIFI
	ESP_ERROR_CHECK(nvs_flash_init());
	wifi.ConfigureWifi();
	wifi.ConnectWifi(WIFI_SSID, WIFI_PASS);	
	vTaskDelay(pdMS_TO_TICKS(5000));
	
	
	//WEBSOCKET	
	websocket.ConfigureWebsocket(WEBSOCKET_URI);

	// IR RECEIVER
	IRSensor.ConfigureIRReceiver();
	mosfet.UpdateGPIO(true);
	
	// MPU E ENCODERS
	mpu.ConfigureMPU();
	encoderL.ConfigureAS5600(pwm_capture_channel_callback);
	
	
	//xTaskCreatePinnedToCore(countencoder, "Encoder Counter Task", 4096, NULL, 1, &xHandleTaskCalculeAngleEncoder, 1);
	vTaskDelay(pdMS_TO_TICKS(3000));

	//xTaskCreatePinnedToCore(calibration, "Calibration", 4096, NULL, 24, &xHandleCalibration, 0);

}
