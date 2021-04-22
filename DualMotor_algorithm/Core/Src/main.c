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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD 180.0/3.14159
#define RAD_REC 3.14159/180.0 
#define SPEED (double)adcArr[0]
#define STEER (double)adcArr[1]

#define MASS 300.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//const
const double Cr=(300*180)/3.14159;
const double Cf=(300*180)/3.14159;
const double Lf=1.503*0.55;
const double Lr=1.503*0.45;
const double Iz=25.0;

//var
uint16_t adcArr[3];
double beta_diff_cur=0.0;
double beta_diff_pre=0.0;
double gamma_diff_cur=0.0;
double gamma_diff_pre=0.0;
double beta=0.0;
double gamma=0.0;

double a11_var;
double a12_var;
double a21_var;
double a22_var;
double b11_var;
double b12_var;

double steer_map=0.0;
double speed_map=0.0;


//time
double counter_cur=0;
double counter_pre=0;
double period=0;

//const double mass=300.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
double map(double value,double inputL,double inputH,double outputL ,double outputH);
double a11(void);
double a12(void);
double a21(void);
double a22(void);
double b11(void);
double b12(void);
double beta_diff(void);
double gamma_diff(void);



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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcArr,3);
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
	a11_var=a11();
	a12_var=a12();
	a21_var=a21();
	a22_var=a22();
	b11_var=b11();
	b12_var=b12();
	speed_map=map((double)adcArr[0],0.0,4095.0,-20.0,20.0);
	steer_map=map((double)adcArr[1],0.0,4095.0,-30*RAD_REC,30*RAD_REC);
	beta_diff_cur=beta_diff();
	gamma_diff_cur = gamma_diff();
	counter_cur=HAL_GetTick();
	period = counter_cur-counter_pre;
	beta+=period*beta_diff()/1000;
	gamma+=period*gamma_diff()/1000;
	counter_pre=counter_cur;
	
	
	
}



double map(double value, double inputL,double inputH,double outputL ,double outputH){
	
	double returnVal=(value-inputL)*(outputH-outputL)/(inputH-inputL)+outputL;
	if(returnVal>outputH){
		return outputH;
	}else {
		return returnVal;
	}
}

double a11(void){
	return -2*(Cr+Cf)/(MASS*speed_map);
}

double a12(void){
	return -1-2*(Lf*Cf-Lr*Cr)/(MASS*speed_map*speed_map);
}

double a21(void){
	return -2*(Lf*Cf-Lr*Cr)/Iz;
}

double a22(void){
	return -2*(Lf*Lf*Cf+Lr*Lr*Cr)/(Iz*speed_map);
}

double b11(void){
	return 2*Cf/(MASS*speed_map);
}

double b12(void){
	return 2*Lf*Cf/Iz;
}

double beta_diff(void){
	return a11()*beta+a12()*gamma+b11()*steer_map;
}

double gamma_diff(void){
	return a21()*beta+a22()*gamma+b12()*steer_map; 
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


