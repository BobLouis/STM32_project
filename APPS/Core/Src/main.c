/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//pinmode setting
#define PedalsLedPin GPIOA,GPIO_PIN_5
#define ReadyLedPin GPIOA,GPIO_PIN_6
#define FaultLedPin GPIOA,GPIO_PIN_7
#define ReadyButtonPin GPIOE,GPIO_PIN_2

//pedal box parameter setting
#define BrakeAct 20
#define APPSAct 50
#define APPSLeast 10   //for the APPS plausibility check  => motor must shut down until the APPS signals less than 5% peadal travel
#define APPSDIFF 50
#define APPSGain 2
#define APPSRUPPER 200
#define APPSLUPPER 200
#define BPPSUPPER 100
#define APPSRLOWER 0
#define APPSLLOWER 0
#define BPPSLOWER 0
#define DISCONNECT 250
//
#define ReadyTime 1000   //ms
//
#define APPSR adcArr[0]
#define APPSL adcArr[1]
#define BPPS   adcArr[2]
#define APPS (adcArr[0]+adcArr[1])/2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void driving_mode(void);
void led_blink(void);
void pedals_mode(void);
uint8_t check_safety(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t adcArr[3];
uint8_t readyButton;
uint8_t errorNumber;
uint8_t appsAv;
uint16_t torque=0;
uint32_t startTime;
uint32_t duration;
bool pedals=0;
bool rtd_io=0;
//boolean readyButton;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	GPIO_PinState readyButton;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcArr,3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		readyButton = HAL_GPIO_ReadPin(ReadyButtonPin);
		//APPSR=adcArr[0]*100/255;
		//APPSL=adcArr[1]*100/255;
		//BPPS =adcArr[2]*100/255;
		//HAL_GPIO_WritePin(ReadyLedPin,GPIO_PIN_SET);
		//HAL_GPIO_WritePin(FaultPin,readyButton);
		
		if(rtd_io==0){
				HAL_GPIO_WritePin(ReadyLedPin,GPIO_PIN_RESET);
				duration=0;
				do{
						if(BPPS>=BrakeAct &&  HAL_GPIO_ReadPin(ReadyButtonPin)==1){
								startTime=HAL_GetTick();
								while(duration<ReadyTime &&   HAL_GPIO_ReadPin(ReadyButtonPin)==1){
										duration = HAL_GetTick()-startTime;
								}
						}
				}while(duration<ReadyTime);
				 rtd_io=1;
			}
			
			if(rtd_io==1){
					HAL_GPIO_WritePin(FaultLedPin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(ReadyLedPin,GPIO_PIN_SET);
				  //entering drive mode
					driving_mode();		
					
			}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void led_blink(void){
	HAL_GPIO_WritePin(ReadyLedPin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(FaultLedPin,GPIO_PIN_SET);
}
void driving_mode(void){
  errorNumber= check_safety();
	switch(errorNumber){
		case 1:{
			//wiring disconnection
			HAL_GPIO_WritePin(FaultLedPin,GPIO_PIN_SET);
			torque=0;
			rtd_io=0;
			break;
		}
		case 2:{
			//apps throttle differ over
			HAL_GPIO_WritePin(FaultLedPin,GPIO_PIN_SET);
			torque=0;
			rtd_io=0;
			break;
		}
		case 3:{
			//bpps || apps out of range
			HAL_GPIO_WritePin(FaultLedPin,GPIO_PIN_SET);
			torque=0;
			rtd_io=0;
			break;
		}
		case 4:{
			//pedals =>enter pedals mode
			HAL_GPIO_WritePin(PedalsLedPin,GPIO_PIN_SET);
			torque=0;
			while(APPS>APPSLeast){
				HAL_Delay(10);
			}
			HAL_GPIO_WritePin(PedalsLedPin,GPIO_PIN_RESET);
			break;
		}
		case 5:{
			//brake acting
			torque=0;
			break;
		}
		case 0:{
			//accelaration mode
			torque=APPS*APPSGain;
			break;
		}
			
	}
	
}
uint8_t check_safety(void){
		//detect disconnect
		if(APPSR>DISCONNECT || APPSL>DISCONNECT || BPPS>DISCONNECT){
			return 1;
		}
		//apps difference over APPS different  limit
		if( __fabs (APPSR-APPSL)> APPSDIFF){
		return 2;
		}
		//check if sensor in the correct zone
		//BPPS 0~100 APPS 0~200
		if(APPSR>APPSRUPPER || APPSL>APPSLUPPER || BPPS>BPPSUPPER||APPSR<APPSRLOWER || APPSL<APPSLLOWER ||BPPS<BPPSLOWER){
			return 3;
		}
		//braking >20
		if(BPPS>BrakeAct){
			if(APPS>APPSAct){
				 pedals=1;	
				return 4;	 //braking with pedal not safe
			}
			return 5;   //braking safely
		}
		return 0;   //accelaration mode
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
