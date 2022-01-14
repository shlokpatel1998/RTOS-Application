/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "usbd_cdc_if.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "spi.h"
#include "i2c.h"
#include "camera.h"
#include "picojpeg.h"
/* USER CODE END Includes */


/* Variables -----------------------------------------------------------------*/
osMessageQId myQueue01Handle;
osMutexId myMutex01Handle;
osSemaphoreId ardCamSemHandle;

/* USER CODE BEGIN Variables */
uint8_t RxData[256];
uint32_t data_received;
uint8_t Str4Display[50];
osThreadId TaskGUIHandle;
osThreadId TaskCOMMHandle;
osThreadId TaskBTNHandle;
osMessageQId CommQueueHandle;
osMutexId dataMutexHandle;

osThreadId TaskIRHandle;
osThreadId TaskArdCamHandle;

int comm_flag = 1;
int sensor_flag = 0;
int secsys_enable = 1;

//For image capture and stuff
//Pulled from jpg2tga.c from the picojpeg-master repo: https://github.com/richgel999/picojpeg


typedef struct
{
	uint8_t Value[10];
	uint8_t Source;
}data;

data DataVCP={"VCP\0",2};

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void StartTaskCOMM(void const * argument);
void StartTaskBTN(void const * argument);
void StartTaskGUI(void const * argument);
void StartTaskIR(void const * argument);
void StartTaskArdCam(void const * argument);
void toggle_enable_system(void);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {

	osMutexDef(dataMutex);
	dataMutexHandle = osMutexCreate(osMutex(dataMutex));

	osSemaphoreDef(ardCamSem);
	ardCamSemHandle = osSemaphoreCreate(osSemaphore(ardCamSem), 1);


	osThreadDef(TaskCOMM, StartTaskCOMM, osPriorityHigh, 0, 128);
	TaskCOMMHandle = osThreadCreate(osThread(TaskCOMM), NULL);

	osThreadDef(TaskBTN, StartTaskBTN, osPriorityAboveNormal, 0, 128);
	TaskBTNHandle = osThreadCreate(osThread(TaskBTN), NULL);

	osThreadDef(TaskGUI, StartTaskGUI, osPriorityAboveNormal, 0, 128);
	TaskGUIHandle = osThreadCreate(osThread(TaskGUI), NULL);

	osThreadDef(TaskIR, StartTaskIR, osPriorityNormal, 0, 128);
	TaskIRHandle = osThreadCreate(osThread(TaskIR), NULL);

	osThreadDef(TaskArdCam, StartTaskArdCam, osPriorityNormal, 0, 128);
	TaskArdCamHandle = osThreadCreate(osThread(TaskArdCam), NULL);

	osMessageQDef(myQueue01, 1, data);
	myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

	/* Comm QUEUE, don't delete */
	osMessageQDef(CommQueue, 1, &DataVCP);
	CommQueueHandle = osMessageCreate(osMessageQ(CommQueue), NULL);


}

void StartTaskCOMM(void const * argument)
{

  osEvent vcpValue;
  osSemaphoreWait(ardCamSemHandle, osWaitForever);

  while(1)
  {
	vcpValue = osMessageGet(CommQueueHandle, osWaitForever);
	osMutexWait(dataMutexHandle, 0);
	memcpy(Str4Display,(char *)(((data *)vcpValue.value.p)->Value), data_received+1);
	osMutexRelease(dataMutexHandle);
	printf("CommTask received: %s\r\n", Str4Display);

	// if received 'c', capture a picture from camera
	if (Str4Display[0] == 'c' && (!sensor_flag && secsys_enable))
	{
		printf("Starting manual photo!\r\n");
		osSemaphoreRelease(ardCamSemHandle);
		osDelay(5000);
		comm_flag = 0;
	}
	if (Str4Display[0] == 'q')
	{
		toggle_enable_system();
	}
  }
}

void StartTaskIR(void const * argument)
{

  while(1)
  {
	if(sensor_flag > 0){
		printf("IR detection\r\n");
		if(cameraReady && secsys_enable){
			osSemaphoreRelease(ardCamSemHandle);
			osDelay(10000); //10 seconds
		}
		sensor_flag = 0;
	}
	osDelay(1000);
  }
}

void StartTaskArdCam(void const * argument)
{
	camera_setup();//init camera
	osDelay(5000);
	while(1)
	{
		if(cameraReady){
			osSemaphoreWait(ardCamSemHandle, osWaitForever);
			camera_initiate_capture();
			osDelay(1000);
		}
	}
}

void StartTaskGUI(void const * argument)
{
	uint8_t buf[sizeof(Str4Display)];
	int flashFlag = 0;
	while(1){
		if(secsys_enable){
			if(sensor_flag){
				flashFlag++;
				flashFlag %= 2;
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				if(flashFlag){
					BSP_LCD_DisplayStringAtLine(11, (uint8_t *)"INTRUDER DETECTED");
				}else{
					BSP_LCD_ClearStringLine(11);
				}
				comm_flag = 0;
			}else if(!comm_flag){
				BSP_LCD_Clear(LCD_COLOR_WHITE);
				comm_flag = 1;
			}else{
				//do nothing
			}
		}else{
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAtLine(7, (uint8_t *)" SECURITY SYSTEM ");
			BSP_LCD_DisplayStringAtLine(8, (uint8_t *)"   IS DISABLED   ");
		}
		if(Str4Display[0]!='\0')
		{
			osMutexWait(dataMutexHandle, 0);
			memcpy(buf,Str4Display, data_received+1);
			Str4Display[0] = '\0';
			osMutexRelease(dataMutexHandle);
			/*BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_ClearStringLine(9);
			BSP_LCD_DisplayStringAtLine(9, buf);*/
			printf("GUI task received: %s\r\n", buf);
		}

		osDelay(500);
	}
}

void StartTaskBTN(void const * argument)
{
  //uint8_t count_press = 0;

  while(1)
  {
	if(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN) == GPIO_PIN_SET){
		//printf("TaskBTN Button Pressed : %i times \n\r", count_press);
		//count_press++;
		toggle_enable_system();
		while(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN)==GPIO_PIN_SET);
	}
	osDelay(10);
  }
}

void toggle_enable_system(void){
	secsys_enable++;
	secsys_enable %= 2;
	if(secsys_enable){
		printf("Enabling security system.\r\n");
		BSP_LCD_Clear(LCD_COLOR_WHITE);
	}else{
		printf("Disabling security system. Stay safe!\r\n");
		BSP_LCD_Clear(LCD_COLOR_RED);
	}
}

