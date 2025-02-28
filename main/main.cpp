#include <iostream>
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
#include "adc/adccontinuos.h"
#include "encoder/encoder.h"

#define DSHOT_MOTOR_R 13

#define DSHOT_MOTOR_L 14
#define MOTOR_MAX_SPEED 165
#define MOTOR_BASE_SPEED 90
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

// for SIRC protocol
//const int start_key = 1, stop_key = 2;
//bool protocol = 1;

IRReceiver IRSensor(static_cast<gpio_num_t>(IR_RECEIVER_PIN), protocol);

float adcSensors[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

int ADC_DATA[6] = {0 , 0, 0, 0, 0,0};

const float kp = 1.6, kd = 1.2, max_accel = 1;

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
		
		//Left
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
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void followLine(void *parameters) {
	static float error = 0, lastError = 0, sensorSum = 0 ;
	static int acel = 0;
	static bool start = false;
	mosfet.UpdateGPIO(true);
	for(;;) {  
		if (command == start_key) start = true;
		
		if (start) {
		for(uint8_t i=0; i<6; i++) {
			adcSensors[i] = adc.ReadAdc(i);
		}
		lastError = error;
		sensorSum = adcSensors[0] + adcSensors[1] + adcSensors[2] + adcSensors[3]+ adcSensors[4] + adcSensors[5];
		
	 	if (error > -5.0 && error < 5.0 && acel < max_accel) {
			 acel++;
		 } else {
			 acel--;
		 }
		 if (acel > max_accel) {
			  acel = max_accel;
		 }
		 if (acel < 0) {
			 acel = 0;
		 }
		 
		if(sensorSum < 5) {
			
	    	if(lastError > 0){
				motorR.UpdateThrottle(MOTOR_BASE_SPEED);
	   		    motorL.UpdateThrottle(20);
			}else{
				motorL.UpdateThrottle(MOTOR_BASE_SPEED);
	   			motorR.UpdateThrottle(20);
			}
			
		} else {
			error = (adcSensors[1] - adcSensors[0]) + 2*(adcSensors[3] - adcSensors[2]) + 3*(adcSensors[5] - adcSensors[4]);
	    	float controllerResult = kp*error + kd*(error - lastError);
			motorR.UpdateThrottle(MOTOR_BASE_SPEED + controllerResult + acel);
	   		motorL.UpdateThrottle(MOTOR_BASE_SPEED - controllerResult + acel);
		}
		
		if (command == stop_key) start = false;
	  } else {
		motorR.UpdateThrottle(0);	
	    motorL.UpdateThrottle(0);
	  }
	    vTaskDelay(pdMS_TO_TICKS(2));
	}
}

void irmonitor(void *parameters) {
	for(;;) {
		command = IRSensor.ReadReceiver();
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void calibration(void *parameters) {
	for(;;) {
		mosfet.UpdateGPIO(true);
		ledWhite.UpdateGPIO(true);
		int speed = MOTOR_BASE_SPEED + 15;
		for(int i = 0; i < 1000; i++) {
			for(int j = 0; j < 9; j++) {
				motorR.UpdateThrottle(speed);
			   	motorL.UpdateThrottle(-speed);
			   	adc.Calibration(j);
		   	}
		}
		motorR.UpdateThrottle(0);
		motorL.UpdateThrottle(0);
		ledWhite.UpdateGPIO(false);
		
		xTaskCreatePinnedToCore(followLine, "FollowLine", 4096, NULL, 2, &xHandleFollowLine, 0);
		//xTaskCreatePinnedToCore(countCheckpoint, "CountCheckpoints", 4096, NULL, 2, &xHandleCountCheckpoint, 1);
		xTaskCreatePinnedToCore(irmonitor, "IRMonitor", 4096, NULL, 2, &xHandleIRMonitor, 1);
		//xTaskCreatePinnedToCore(readEncoder, "ReadEncoder", 4096, NULL, 2, &xHandleReadEncoder, 1);

		//vTaskSuspend(xHandleCalibration);
		vTaskDelete(NULL);
	}
}

static bool IRAM_ATTR callback(adc_continuous_ctx_t *handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t mustYield = pdFALSE;
    //vTaskNotifyGiveFromISR(cb_task, &mustYield);
    vTaskNotifyGiveFromISR(xHandleCalibration, &mustYield);
    return (mustYield == pdTRUE);  
}
int countCalibration = 0;

void calibration2(void *parameters) {
	for(;;) {
		
		mosfet.UpdateGPIO(true);
		ledWhite.UpdateGPIO(true);
		int speed = MOTOR_BASE_SPEED + 15;		
		motorR.UpdateThrottle(speed);
	   	motorL.UpdateThrottle(-speed);
	   	adcContinuos.Calibration();	
		countCalibration++;
		if(countCalibration > 2000){
			
			printf("stop calibracao\n\n\n");
			motorR.UpdateThrottle(0);
			motorL.UpdateThrottle(0);
			ledWhite.UpdateGPIO(false);		
			adcContinuos.GetMinAndMaxValues();
			vTaskSuspend(xHandleCalibration);
			vTaskDelete(NULL);
			
			//xTaskCreatePinnedToCore(followLine, "FollowLine", 4096, NULL, 2, &xHandleFollowLine, 0);
			//xTaskCreatePinnedToCore(countCheckpoint, "CountCheckpoints", 4096, NULL, 2, &xHandleCountCheckpoint, 1);
			//xTaskCreatePinnedToCore(irmonitor, "IRMonitor", 4096, NULL, 2, &xHandleIRMonitor, 1);
			//xTaskCreatePinnedToCore(readEncoder, "ReadEncoder", 4096, NULL, 2, &xHandleReadEncoder, 1);
		}
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}
}


void cbTask (void *parameters)
{
	
	for (;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		adcContinuos.ReadAdc(&ADC_DATA);		
		printf("s1: %d ; s2: %d ;  s3: %d ;  s4: %d ; s5: %d ; s6: %d ; ", ADC_DATA[4], ADC_DATA[2], ADC_DATA[0], ADC_DATA[1],ADC_DATA[3],ADC_DATA[5]);
		printf("\n");
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
	xTaskCreatePinnedToCore(calibration2, "Calibration", 4096, NULL, 24, &xHandleCalibration, 0);

	//xTaskCreate(cbTask, "Callback Task", 4096, NULL, 0, &cb_task);
	
	
	//xTaskCreatePinnedToCore(readAdcContinuos, "AdcContinuos", 4096, NULL, 24, &xHandleAdcContinuos, 0);
}
