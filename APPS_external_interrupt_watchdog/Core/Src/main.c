/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//pedal box parameter setting
#define BrakeAct 800
#define APPSAct 800
#define APPSLeast 200   //for the APPS plausibility check  => motor must shut down until the APPS signals less than 5% peadal travel
#define APPSDIFF 2000


//if out of this range will generate error
#define APPSRUPPEST 4000
#define APPSLUPPEST 4000
#define BPPSUPPEST 4000
#define APPSRLOWEST 0
#define APPSLLOWEST 0
#define BPPSLOWEST 0
#define DISCONNECT 4050
//deine max and offset this parameter offset will slight bigger than the lowest and max will slight smaller than the uppest depends on  the pedal box
//this parameter is used to determine the output torque
#define APPSRMAX 3800
#define APPSLMAX 3800
#define APPSROFFSET 0
#define APPSLOFFSET 0

#define ReadyTime 2000   //ms

//dma scan
#define APPSR adcArr[0]
#define APPSL adcArr[1]
#define BPPS   adcArr[2]
#define APPS (adcArr[0]+adcArr[1])/2
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static CAN_TxHeaderTypeDef TxMessage;
static CAN_RxHeaderTypeDef RxMessage;

//Rx Txdata
uint8_t TxData[8]={0};
uint32_t TxMailbox;
uint8_t RxData[8]={0};

//address
uint16_t OwnID=0x123;
uint16_t RemoteID =0x0A0;



uint16_t adcArr[3];
bool readyButton;
uint8_t errorNumber;
uint16_t torque=0;
uint32_t startTime;
uint32_t duration=0;
bool pedals=0;
bool rtd_io=0;
bool rtd_start=0; //if precharge&&reset&&readyToDrive io are all on this parameter will be true
//bool ready_io=0;
bool precharge_io=0;
bool reset_io=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
//APPS
void driving_mode(void);
void led_blink(void);
void pedals_mode(void);
uint8_t check_safety(void);
uint16_t map(uint16_t value,uint16_t inputL,uint16_t inputH,uint16_t outputL ,uint16_t outputH);

//CAN
void CAN_filterConfig(void);
void CAN_Txsetup(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcArr,3);
	HAL_TIM_Base_Start_IT(&htim2);
	CAN_filterConfig();
	CAN_Txsetup();
	
	precharge_io=HAL_GPIO_ReadPin(precharge_SW_GPIO_Port,precharge_SW_Pin);
	if(precharge_io)
		HAL_GPIO_WritePin(precharge_LED_GPIO_Port,precharge_LED_Pin,GPIO_PIN_SET);
	reset_io=!(HAL_GPIO_ReadPin(reset_SW_GPIO_Port,reset_SW_Pin));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//waiting for precharge reset_switch, ready to drive to activate the driving mode
		if(rtd_io==0){
				torque=0;
				HAL_GPIO_WritePin(readyToDrive_LED_GPIO_Port,readyToDrive_LED_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(pedals_LED_GPIO_Port,pedals_LED_Pin,GPIO_PIN_RESET);
				duration=0;
				do{
						if(BPPS>=BrakeAct &&  rtd_start==1){
								startTime=HAL_GetTick();
								while(duration<ReadyTime &&   rtd_start==1){
										duration = HAL_GetTick()-startTime;
								}
						}
				}while(duration<ReadyTime);
				 rtd_io=1;
			}
			//driving stage
			if(rtd_io==1){
					HAL_GPIO_WritePin(fault_LED_GPIO_Port,fault_LED_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(readyToDrive_LED_GPIO_Port,readyToDrive_LED_Pin,GPIO_PIN_SET);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* EXTI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim2)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim2);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */

	/*01 torque command (torque = ([0]+[1]*256)/10)
  23 speed command (angular speed = [2]+[3]*256
  4   Direction (0:reverse 1:forwards rd).  *further note: if the direction command is changed suddenly when the inverter is still enable, inverter is disable without triggering any fault
  And the Lockout mechanism is set again which will force the user to re-enable it
  5   5.0 inverter enable (0:off 1:ON)
       5.1 inverter discharge (0 disable 1 enable)
       5.2 speed mode
  67  commanded torque limit (0 default)
  this message should be continuously broadcast at least 500 milliseconds
	*/
	
	if(rtd_io==0){
		TxData[0]=0;
		TxData[1]=0;
		TxData[5]=0;
	}else{
		TxData[0]=(torque%256);
		TxData[1]=(torque/256);
		TxData[4]=1;
		TxData[5]=1;
	}
	HAL_CAN_AddTxMessage(&hcan1,&TxMessage,TxData,&TxMailbox);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_1);
}

void CAN_filterConfig(void){
	CAN_FilterTypeDef filterConfig;
	
	filterConfig.FilterBank=0;
	filterConfig.FilterActivation=ENABLE;
	filterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
	filterConfig.FilterIdHigh=0x0000;
	filterConfig.FilterIdLow=0x0000;
	filterConfig.FilterMaskIdHigh=0x0000;
	filterConfig.FilterMaskIdLow=0x0000;
	filterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale= CAN_FILTERSCALE_32BIT;
	filterConfig.SlaveStartFilterBank=14;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&filterConfig)!=HAL_OK){
		Error_Handler();
	}
	
	if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK){
		Error_Handler();
	}		//enable interrupt
	
}
void CAN_Txsetup(){
		if(HAL_CAN_Start(&hcan1)!=HAL_OK){
		Error_Handler();
		}
		
		TxMessage.StdId=0x0C0;
		TxMessage.ExtId=0x01;
		TxMessage.RTR=CAN_RTR_DATA;
		TxMessage.IDE=CAN_ID_STD;
		TxMessage.DLC=8;
		TxMessage.TransmitGlobalTime=DISABLE;   //time trigger must be turned ON
		
		TxData[0]=0;
		TxData[1]=0;
		TxData[2]=0;
		TxData[3]=0;
		TxData[4]=0;
		TxData[5]=0;
		TxData[6]=0;
		TxData[7]=0;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMessage,RxData)!=HAL_OK){
		Error_Handler();
	}
}


void driving_mode(void){
  errorNumber= check_safety();
	switch(errorNumber){
		case 1:{
			//wiring disconnection
			HAL_GPIO_WritePin(fault_LED_GPIO_Port,fault_LED_Pin,GPIO_PIN_SET);
			torque=0;
			rtd_io=0;
			break;
		}
		case 2:{
			//apps throttle differ over
			HAL_GPIO_WritePin(fault_LED_GPIO_Port,fault_LED_Pin,GPIO_PIN_SET);
			torque=0;
			rtd_io=0;
			break;
		}
		case 3:{
			//bpps || apps out of range
			HAL_GPIO_WritePin(fault_LED_GPIO_Port,fault_LED_Pin,GPIO_PIN_SET);
			torque=0;
			rtd_io=0;
			break;
		}
		case 4:{
			//pedals =>enter pedals mode
			HAL_GPIO_WritePin(pedals_LED_GPIO_Port,pedals_LED_Pin,GPIO_PIN_SET);
			torque=0;
			pedals=1;
			break;
		}
		case 5:{
			//brake acting
			torque=0;
			break;
		}
		case 0:{
			//accelaration mode
			
			torque=map(APPS,(APPSROFFSET+APPSLOFFSET)/2,(APPSRMAX+APPSLMAX)/2,0,1400);
			break;
		}
			
	}
	if(pedals==1){
		if(APPS<APPSLeast){
			pedals=0;
			HAL_GPIO_WritePin(pedals_LED_GPIO_Port,pedals_LED_Pin,GPIO_PIN_RESET);
		}
	}
}
uint8_t check_safety(void){
		//detect disconnect
		if(APPSR>DISCONNECT || APPSL>DISCONNECT || BPPS>DISCONNECT){
			return 1;
		}
		//apps difference over APPS different  limit
		else if( __fabs (APPSR-APPSL)> APPSDIFF){
			return 2;
		}
		//check if sensor in the correct zone
		//BPPS 0~100 APPS 0~200
		else if(APPSR>APPSRUPPEST || APPSL>APPSLUPPEST || BPPS>BPPSUPPEST||APPSR<APPSRLOWEST || APPSL<APPSLLOWEST ||BPPS<BPPSLOWEST){
			return 3;
		}
		//braking >20
		else if(BPPS>BrakeAct){
			if(APPS>APPSAct){
				 pedals=1;	
				return 4;	 //braking with pedal not safe
			}
			return 5;   //braking safely
		}
		return 0;   //accelaration mode
}

uint16_t map(uint16_t value, uint16_t inputL,uint16_t inputH,uint16_t outputL ,uint16_t outputH){
	
	uint16_t returnVal=(value-inputL)*(outputH-outputL)/(inputH-inputL)+outputL;
	if(returnVal>outputH){
		return outputH;
	}else {
		return returnVal;
	}
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
