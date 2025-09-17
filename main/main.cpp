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

#define DSHOT_MOTOR_R 13

#define DSHOT_MOTOR_L 14
#define MOTOR_MAX_SPEED 90
#define MOTOR_BASE_SPEED 50
#define GPIO_MOSFET 23
#define GPIO_LED1 17
#define GPIO_LED2 5
/*#define GPIO_ENCR_CHA 18
#define GPIO_ENCR_CHB 19
#define GPIO_ENCL_CHA 16
#define GPIO_ENCL_CHB 4*/
#define IR_RECEIVER_PIN 15
#define GPIO_MPU_SDA 22
#define GPIO_MPU_SCL 21
#define I2C_PORT I2C_NUM_0
#define AS5600_SDA_PIN  15
#define AS5600_SCL_PIN  2
#define AS5600_I2C_PORT I2C_NUM_1 

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
TaskHandle_t xHandleReadEncoder = NULL;
TaskHandle_t xHandleIRMonitor = NULL;
TaskHandle_t xHandleReadMpuEncoder = NULL;


TaskHandle_t cb_task = NULL;
			 
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


As5600 encoderL(static_cast<gpio_num_t>(AS5600_SDA_PIN), static_cast<gpio_num_t>(AS5600_SCL_PIN),static_cast<i2c_port_num_t>(AS5600_I2C_PORT));
As5600 encoderR(static_cast<gpio_num_t>(13), static_cast<gpio_num_t>(12),static_cast<i2c_port_num_t>(AS5600_I2C_PORT));
Mpu mpu(static_cast<gpio_num_t>(GPIO_MPU_SDA),static_cast<gpio_num_t>(GPIO_MPU_SCL) , static_cast<i2c_port_num_t>(I2C_PORT));
Wifi wifi;
Websocket websocket;

Gpio encoderA(static_cast<gpio_num_t>(18));
Gpio encoderB(static_cast<gpio_num_t>(19));


// for NEC protocol
const int start_key = 69, stop_key = 70;
bool protocol = 0;


IRReceiver IRSensor(static_cast<gpio_num_t>(IR_RECEIVER_PIN), protocol);

float adcSensors[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float adcSideSensors[2] = {0.0, 0.0};


const float kp = 1.10, kd = 0.0, max_accel = 1;

uint8_t command;
static int countR = 0, countL= 0;



void readMpuEncoder(void *parameters){
	printf("readMPU");
	for(;;){
		int gyroX, gyroY, gyroZ, accelX, accelY, accelZ = 0;
		mpu.ReadMPU(&gyroX, &gyroY, &gyroZ, &accelX, &accelY, &accelZ);
		int16_t angleL = encoderL.ReadRawAngle();		
		websocket.SendData(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, angleL, angleL);
		vTaskDelay(pdMS_TO_TICKS(1));
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
		//adc.Calibration(0);
		//adc.Calibration(1);
		//adc.Calibration(2);
		if(countCalibration > 20000){
			
			printf("stop calibracao\n\n\n");
			motorR.UpdateThrottle(0);
			motorL.UpdateThrottle(0);
			ledWhite.UpdateGPIO(false);		
			adcContinuos.GetMinAndMaxValues();
			//adc.GetMinAndMaxValues();
			
			
			xTaskCreatePinnedToCore(followLine, "FollowLine", 4096, NULL, 2, &xHandleFollowLine, 0);
			xTaskCreatePinnedToCore(readLine, "ReadLine", 4096, NULL, 2, &xHandleReadLine, 0);
			
		
			//xTaskCreatePinnedToCore(sendData, "SendData", 4096, NULL, 2, &xHandleSendData, 1);
			xTaskCreatePinnedToCore(irmonitor, "IRMonitor", 4096, NULL, 2, &xHandleIRMonitor, 1);
			xTaskCreatePinnedToCore(readMpuEncoder, "ReadMPU", 4096, NULL, 2, &xHandleReadMpuEncoder, 1);
			//xTaskCreatePinnedToCore(countCheckpoint, "CountCheckpoints", 4096, NULL, 2, &xHandleCountCheckpoint, 1);
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
	
	//ledWhite.ConfigureGPIO();
	//ledYellow.ConfigureGPIO();	
	//motorL.ConfigureDshot();	
	//motorR.ConfigureDshot();	
	//vTaskDelay(pdMS_TO_TICKS(5000));
	//mosfet.ConfigureGPIO();
	//adc.ConfigureAdc();
	
	//WIFI
	ESP_ERROR_CHECK(nvs_flash_init());
	wifi.ConfigureWifi();
	wifi.ConnectWifi(WIFI_SSID, WIFI_PASS);
	
	
	vTaskDelay(pdMS_TO_TICKS(5000));
	//WEBSOCK	
	websocket.ConfigureWebsocket(WEBSOCKET_URI);

	
	/*IRSensor.ConfigureIRReceiver();*/
	//mosfet.UpdateGPIO(true);
	
	//adcContinuos.ConfigureAdc(callback);
	mpu.ConfigureMPU();
	encoderL.ConfigureAS5600();
	vTaskDelay(pdMS_TO_TICKS(5000));
	
	
	xTaskCreatePinnedToCore(readMpuEncoder, "ReadMPU", 4096, NULL, 2, &xHandleReadMpuEncoder, 1);
	
	//
	//xTaskCreatePinnedToCore(calibration, "Calibration", 4096, NULL, 24, &xHandleCalibration, 0);

}
