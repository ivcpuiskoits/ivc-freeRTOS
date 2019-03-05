/* USER CODE BEGIN Header */
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
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include <stdbool.h>
#include "sim900A.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim14;

extern uint32_t a,b,c,d;

/* CAN variables */
extern CAN_TxHeaderTypeDef   TxHeader;
extern CAN_RxHeaderTypeDef   RxHeader;
extern uint8_t               TxData[8];
extern uint8_t               RxData[8];
extern uint32_t              TxMailbox;

// BMS variable
extern int supply12V, packSOC, ampHours, lowestVolt;
extern uint32_t totalVolt;
extern bool right, left, lamp;
extern float coloumb;
extern uint16_t current;
extern uint16_t temperature[4];

// controller variable
extern uint32_t counter;
extern uint16_t motorCurrent;
extern uint16_t controllerTemperature;
extern uint16_t rpm;
extern uint8_t errorCode;
extern uint16_t motorTemperature;
extern uint16_t d_battCurrentController;
extern uint16_t totalVoltController;

// pulser variable
extern uint16_t counterL;
extern uint8_t counterH;
extern float periode, freq;

extern int mode;

uint8_t buffSerial[16];
uint8_t serialIndex;
bool flagEngine = true;
int count, state;


extern uint8_t rx_data, rx_buffer[256], rx_index, Transfer_cplt;
extern char tx_buffer[512];
extern float lastdata;

/* bluetooth variables */
extern uint8_t rx_data2, rx_buffer2[256], Transfer_cplt2;
extern uint32_t rx_index2;
extern char tx_buffer2[512];

extern uint32_t n;


/* USER CODE END Variables */
osThreadId androidTaskHandle;
osThreadId gsmTaskHandle;
osThreadId canTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
unsigned char ascii_to_hex(unsigned char data);
void input_handler(void);
void Clr_Buff2();   
/* USER CODE END FunctionPrototypes */

void StartAndroidTask(void const * argument);
void StartGsmTask(void const * argument);
void StartCanTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of androidTask */
  osThreadDef(androidTask, StartAndroidTask, osPriorityNormal, 0, 512);
  androidTaskHandle = osThreadCreate(osThread(androidTask), NULL);

  /* definition and creation of gsmTask */
  osThreadDef(gsmTask, StartGsmTask, osPriorityNormal, 0, 512);
  gsmTaskHandle = osThreadCreate(osThread(gsmTask), NULL);

  /* definition and creation of canTask */
  osThreadDef(canTask, StartCanTask, osPriorityNormal, 0, 512);
  canTaskHandle = osThreadCreate(osThread(canTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartAndroidTask */
/**
  * @brief  Function implementing the androidTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartAndroidTask */
void StartAndroidTask(void const * argument)
{

  /* USER CODE BEGIN StartAndroidTask */
  /* Infinite loop */
  for(;;)
  {
		if (state == 0) // BMS Parameters
		{
//			cnt=500;
//				lowestVolt = 2725;
//				current = 3535;
//				totalVolt = 500000;
//				packSOC += 10; 
				if (packSOC > 100) 
				{packSOC = 0;} 
//				lowestVolt += 50;
//				if (lowestVolt == 5000)
//					lowestVolt = 0;
//				current = 3535;
//				temperature[0] += 100;
//				if (temperature[0] == 5000)
//					temperature[0] = 0;
			temperature[1] = temperature[0];
			temperature[2] = temperature[0];
			temperature[3] = temperature[0];
//			cnt=501;
			Clr_Buff2();
//			cnt=502;
			n=sprintf(tx_buffer2,"(%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X)\n\r",
							 state, packSOC, lowestVolt >> 8, lowestVolt & 0xFF, current >> 8, current & 0xFF,
							 temperature[0] >> 8, temperature[0] & 0xFF, temperature[1] >> 8, temperature[1] & 0xFF,
							 temperature[2] >> 8, temperature[2] & 0xFF, temperature[3] >> 8, temperature[3] & 0xFF);
			HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer2, n, 100);
//				printf("(%02X %02X %02X %02X %02X %02X %02X "
//			          "%02X %02X %02X %02X %02X %02X %02X)\n\r",
//							   state, packSOC, lowestVolt >> 8, lowestVolt & 0xFF, current >> 8, current & 0xFF,
//							   temperature[0] >> 8, temperature[0] & 0xFF, temperature[1] >> 8, temperature[1] & 0xFF,
//							   temperature[2] >> 8, temperature[2] & 0xFF, temperature[3] >> 8, temperature[3] & 0xFF);
//				if (packSOC > 100) packSOC = 100;
//			cnt=503;
			state = 1;
//			cnt=504;
		}
		else if (state == 1) // Controller Parameters
		{
//				controllerTemperature = 35;
//				motorTemperature = 25;
//				rpm = 2525;
////				rpm += 100;
//				if (rpm > 5000) rpm = 0;
//				motorCurrent = 157;
//				d_battCurrentController = 3575;
//				errorCode = 0;
//				totalVolt += 5;
//				if (totalVolt > 255)
//					totalVolt = 0;
			Clr_Buff2();
			n=sprintf(tx_buffer2,"(%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X)\n\r",
							 state,rpm >> 8, rpm & 0xFF, motorCurrent >> 8, motorCurrent & 0xFF, controllerTemperature >> 8, controllerTemperature & 0xFF,
								motorTemperature >> 8, motorTemperature & 0xFF, errorCode, d_battCurrentController >> 8, d_battCurrentController & 0xFF, 
								totalVoltController >> 8, totalVoltController & 0xFF);
			HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer2, n, 100);
//				printf("(%02X %02X %02X %02X %02X %02X %02X "
//								"%02X %02X %02X %02X %02X %02X %02X)\n\r",
//				         state,rpm >> 8, rpm & 0xFF, motorCurrent >> 8, motorCurrent & 0xFF, controllerTemperature >> 8, controllerTemperature & 0xFF,
//									motorTemperature >> 8, motorTemperature & 0xFF, errorCode, d_battCurrentController >> 8, d_battCurrentController & 0xFF, 
//									totalVoltController >> 8, totalVoltController & 0xFF);
			rpm++;
			motorCurrent++;
			state = 2;
		}
		else if (state == 2) // Miscellaneous
		{
//				right = !right;
//				lamp = !lamp;
//				left = !left;
//				right = 0;
			Clr_Buff2();
			n=sprintf(tx_buffer2,"(%02X %02X %02X %02X)\n\r",state, right, left, lamp);
			HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer2, n, 100);
//				printf("(%02X %02X %02X %02X)\n\r",state, right, left, lamp);
			if (mode == 0x00)
				state = 0;
			else if (mode == 0x01)
				//state = 3;
			{	packSOC += 10; if (packSOC > 100) packSOC = 0; }
		}
		else if (state == 3) //-- state 3 to control engine on/off
		{
			if (buffSerial[0] == 12)
			{
				flagEngine = buffSerial[1];
				uint8_t id = 0xFF; //-- edit to change CAN ID
//					TxMessage.ExtId = (uint32_t)(id << 8) | IVC_ID
//					TxMessage.RTR = CAN_RTR_DATA;
//					TxMessage.IDE = CAN_ID_EXT;
//					TxMessage.DLC = 8;
//					TxMessgae.Data[0] = flagEngine;
//					CAN_Transmit(CAN_COM, &TxMessage);
				
				TxHeader.ExtId = (uint32_t)(buffSerial[0] << 8) | IVC_ID;
				TxData[0] = flagEngine;
				HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			}
			
			state = 0;
		}
		//printf("state : %d\n\r",state);
	
		input_handler();
	
		count = 0;
//		cnt=0;
		a++;
		HAL_Delay(1000);
    osDelay(1);
  }
  /* USER CODE END StartAndroidTask */
}

/* USER CODE BEGIN Header_StartGsmTask */
/**
* @brief Function implementing the gsmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGsmTask */
void StartGsmTask(void const * argument)
{
  /* USER CODE BEGIN StartGsmTask */
  /* Infinite loop */
	SIM900A_Init();

  for(;;)
  {
		ReadLastData(683369, 1);
//		power = SIM7000_ReadLastData_tsk(683369, 1);
		TxHeader.ExtId = 0x12 << 8 | IVC_ID;
		TxData[0] = lastdata;
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		b++;
		osDelay(1000);
    osDelay(1);
  }
  /* USER CODE END StartGsmTask */
}

/* USER CODE BEGIN Header_StartCanTask */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTask */
void StartCanTask(void const * argument)
{
  /* USER CODE BEGIN StartCanTask */
  /* Infinite loop */
  for(;;)
  {
		c++;
		vTaskDelay(1000);
    osDelay(1);
  }
  /* USER CODE END StartCanTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
unsigned char ascii_to_hex(unsigned char data)
{
    if ((data >= '0') && (data <= '9' )) {
        // Numbers (0-9)
        data -= '0';
    } else if ((data >= 'A') && (data <= 'F' )) {
        // Uppercase (A-F)
        data = data -'A' + 10;
    } else if ((data >= 'a') && (data <= 'f' )) {
        // Lowercase (a-f)
        data = data -'a' + 10;
    } else {
        // Illegal
        data = 0;
    }

    return data;
}

void input_handler(void)
{
//	static uint16_t count;
//	count++;
//	if (count == 500)
//	{
		if (HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin) == 0) //right
		{			
			// just for debugging
			//	printf("Input 1 is pressed\n\r");
			right = 1;
		} 
		else
		{
			right = 0;
		}
		
		if (HAL_GPIO_ReadPin(IN2_GPIO_Port, IN2_Pin) == 0) //left
		{		
			// just for debugging
			//	printf("Input 2 is pressed\n\r");
			left = 1;
		}
		else
		{
			left = 0;
		}
		
		if (HAL_GPIO_ReadPin(IN3_GPIO_Port, IN3_Pin) == 0) //lamp
		{			
			// just for debugging
			//	printf("Input 3 is pressed\n\r");
			lamp = 1;
		}
		else
		{
			lamp = 0;
		}
		
//		if (GPIO_ReadInputDataBit(INPUT_GPIO, IN4) == Bit_RESET)
//		{
//			
//			// just for debugging
//				printf("Input 4 is pressed\n\r");
//		}
//		else
//		{
//			
//		}
//		
//		count = 0;
//	}
}


void Clr_Buff2()
{
	for(uint8_t i=0;i<=rx_index2;i++)
			rx_buffer2[i]=0;
	
	rx_index2 = 0;
	for(uint8_t x=0;x<=100;x++);
}     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
