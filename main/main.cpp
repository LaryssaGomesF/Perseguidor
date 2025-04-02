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

#define DSHOT_MOTOR_R 13

#define DSHOT_MOTOR_L 14
#define MOTOR_MAX_SPEED 90
#define MOTOR_BASE_SPEED 50
#define GPIO_MOSFET 23
#define GPIO_LED1 17
#define GPIO_LED2 5
#define GPIO_ENCR_CHA 18
#define GPIO_ENCR_CHB 19
#define GPIO_ENCL_CHA 16
#define GPIO_ENCL_CHB 4
#define IR_RECEIVER_PIN 15

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
TaskHandle_t xHandleReadEncoder = NULL;
TaskHandle_t xHandleIRMonitor = NULL;
TaskHandle_t cb_task = NULL;
			 
// Classes presenting the project components
Gpio mosfet(static_cast<gpio_num_t>(GPIO_MOSFET));
Gpio ledWhite(static_cast<gpio_num_t>(GPIO_LED1));
Gpio ledYellow(static_cast<gpio_num_t>(GPIO_LED2));
Dshot motorL(static_cast<gpio_num_t>(DSHOT_MOTOR_L), false, MOTOR_MAX_SPEED);
Dshot motorR(static_cast<gpio_num_t>(DSHOT_MOTOR_R), true, MOTOR_MAX_SPEED);
adc_channel_t adc1[6] = {ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_0, ADC_CHANNEL_5};
adc_channel_t adc2[3] = {ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9};
Adc adc(adc1, adc2, TRACE_COLOR);
AdcContinuos adcContinuos(adc1,  TRACE_COLOR );
Encoder encoderL(GPIO_ENCL_CHA, GPIO_ENCL_CHB, 0);
Encoder encoderR(GPIO_ENCR_CHA, GPIO_ENCR_CHB, 0);


// for NEC protocol
const int start_key = 69, stop_key = 70;
bool protocol = 0;


IRReceiver IRSensor(static_cast<gpio_num_t>(IR_RECEIVER_PIN), protocol);

float adcSensors[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

int ADC_DATA[6] = {0 , 0, 0, 0, 0,0};

const float kp = 1.10, kd = 0.0, max_accel = 1;

uint8_t command;



static int countR = 0, countL= 0;

void readEncoder(void *parameters){
	printf("readEncoder");
	int leftEncoder = 0, rightEncoder = 0;
	for(;;){
		
		encoderL.ReadEncoder(&leftEncoder);
		encoderR.ReadEncoder(&rightEncoder);
		printf("%d - %d", leftEncoder, rightEncoder);
		printf("\n");
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

	
void countCheckpoint(void *parameters){
	static bool stateL = false, stateR = false;
	
	printf("countCheckpoint");
	
	for (;;) {
		
		/*//Left
		adcSensors[7] = adc.ReadAdc(7);
		if(adcSensors[7] > 2.5 && stateL == false){
			
		
			stateL = true;
			countL++;
		} 
		if(adcSensors[7] < 2.5 && stateL == true){
			stateL = false;
			
		}
		
		//Right
		adcSensors[8] = adc.ReadAdc(8);
		if(adcSensors[8] > 2.5 && stateR == false){
			ledWhite.UpdateGPIO(true);
			ledYellow.UpdateGPIO(true);
			stateR = true;
			countR++;
		} 
		if(adcSensors[8] < 2.5 && stateR == true){
			ledWhite.UpdateGPIO(false);
			ledYellow.UpdateGPIO(false);
			stateR = false;
		}*/
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



static bool IRAM_ATTR callback(adc_continuous_ctx_t *handle, const adc_continuous_evt_data_t *edata, void *user_data) {
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



int countCalibration = 0;

void calibration(void *parameters) {
	for(;;) {
		
		mosfet.UpdateGPIO(true);
		ledWhite.UpdateGPIO(true);
		int speed = MOTOR_BASE_SPEED + 15;		
		motorR.UpdateThrottle(speed);
	   	motorL.UpdateThrottle(-speed);
	   	adcContinuos.Calibration();	
		countCalibration++;
		if(countCalibration > 20000){
			
			printf("stop calibracao\n\n\n");
			motorR.UpdateThrottle(0);
			motorL.UpdateThrottle(0);
			ledWhite.UpdateGPIO(false);		
			adcContinuos.GetMinAndMaxValues();
			
			
			xTaskCreatePinnedToCore(followLine, "FollowLine", 4096, NULL, 2, &xHandleFollowLine, 0);
			xTaskCreatePinnedToCore(readLine, "ReadLine", 4096, NULL, 2, &xHandleReadLine, 0);
			
			//xTaskCreatePinnedToCore(countCheckpoint, "CountCheckpoints", 4096, NULL, 2, &xHandleCountCheckpoint, 1);
			xTaskCreatePinnedToCore(irmonitor, "IRMonitor", 4096, NULL, 2, &xHandleIRMonitor, 1);
			//xTaskCreatePinnedToCore(readEncoder, "ReadEncoder", 4096, NULL, 2, &xHandleReadEncoder, 1);
			
			vTaskSuspend(xHandleCalibration);
			vTaskDelete(NULL);
		}
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}
}



extern "C" void app_main(void)
{
	printf("Running Experiment262");
	
	ledWhite.ConfigureGPIO();
	ledYellow.ConfigureGPIO();	
	motorL.ConfigureDshot();	
	motorR.ConfigureDshot();	
	vTaskDelay(pdMS_TO_TICKS(5000));
	mosfet.ConfigureGPIO();
	//adc.ConfigureAdc();
	
	encoderL.ConfigureEncoder();
	encoderR.ConfigureEncoder();	
	IRSensor.ConfigureIRReceiver();
	mosfet.UpdateGPIO(true);
	
	adcContinuos.ConfigureAdc(callback);
	xTaskCreatePinnedToCore(calibration, "Calibration", 4096, NULL, 24, &xHandleCalibration, 0);
	xTaskCreatePinnedToCore(irmonitor, "IRMonitor", 4096, NULL, 1, &xHandleIRMonitor, 1);

}
