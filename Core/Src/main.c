/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stddef.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DIG_1_ON		LL_GPIO_SetOutputPin (GPIOB, LL_GPIO_PIN_9);
#define DIG_3_ON		LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_1);
#define	DIG_4_ON		LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_2);
#define	DIG_2_ON		LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_0);

#define DIG_1_OFF		LL_GPIO_ResetOutputPin (GPIOB, LL_GPIO_PIN_9);
#define	DIG_2_OFF		LL_GPIO_ResetOutputPin (GPIOC, LL_GPIO_PIN_0);
#define DIG_3_OFF		LL_GPIO_ResetOutputPin (GPIOC, LL_GPIO_PIN_1);
#define	DIG_4_OFF		LL_GPIO_ResetOutputPin (GPIOC, LL_GPIO_PIN_2);

#define SEG_D_ON		LL_GPIO_ResetOutputPin (GPIOC, LL_GPIO_PIN_3);//D
#define SEG_E_ON		LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_0);//E
#define SEG_F_ON		LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_1);//F
#define SEG_A_ON		LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_2);//A
#define SEG_B_ON		LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_3);//B
#define SEG_C_ON		LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_4);//C
#define SEG_G_ON		LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_5);//G
#define SEG_D3_ON		LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_6);//D3 + DIG2
#define SEG_D4_ON		LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_7);//D4 + DIG3

#define SEG_D_OFF		LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_3);//D
#define SEG_E_OFF		LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_0);//E
#define SEG_F_OFF		LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_1);//F
#define SEG_A_OFF		LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_2);//A
#define SEG_B_OFF		LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_3);//B
#define SEG_C_OFF		LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_4);//C
#define SEG_G_OFF		LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_5);//G
#define SEG_D3_OFF	LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_6);//D3 + DIG2
#define SEG_D4_OFF	LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_7);//D4 + DIG3
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t timeout = 250;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void blurs_on(void)
{
	DIG_2_ON
	SEG_D3_OFF
	DIG_3_ON
	SEG_D4_OFF
}

void blurs_off(void)
{
	DIG_2_OFF
	SEG_D3_OFF
	DIG_3_OFF
	SEG_D4_OFF
}

void digital_seg_1_on(void)
{
	DIG_1_ON DIG_2_OFF DIG_3_OFF DIG_4_OFF
}

void digital_seg_1_off(void)
{
	DIG_1_OFF
}

void digital_seg_2_on(void)
{
	DIG_1_OFF DIG_2_ON DIG_3_OFF DIG_4_OFF
}

void digital_seg_2_off(void)
{
	DIG_2_OFF
}


void digital_seg_3_on(void)
{
	DIG_1_OFF DIG_2_OFF DIG_3_ON DIG_4_OFF
}

void digital_seg_3_off(void)
{
	DIG_3_OFF
}

void digital_seg_4_on(void)
{
	DIG_1_OFF DIG_2_OFF DIG_3_OFF DIG_4_ON
}

void digital_seg_4_off(void)
{
	DIG_4_OFF
}

void digital_0_on(void)
{
	SEG_A_ON SEG_B_ON	SEG_C_ON SEG_D_ON	SEG_E_ON SEG_F_ON	SEG_G_OFF	SEG_D3_OFF SEG_D4_OFF
}

void digital_0_off(void)
{
	SEG_A_OFF SEG_B_OFF SEG_C_OFF SEG_D_OFF SEG_E_OFF SEG_F_OFF
}

void digital_1_on(void)
{
	SEG_A_OFF	SEG_B_ON SEG_C_ON SEG_D_OFF SEG_E_OFF SEG_F_OFF SEG_G_OFF SEG_D3_OFF SEG_D4_OFF
}

void digital_1_off(void)
{
	SEG_B_OFF SEG_C_OFF
}

void digital_2_on(void)
{
	SEG_A_ON SEG_B_ON SEG_C_OFF SEG_D_ON SEG_E_ON SEG_F_OFF SEG_G_ON SEG_D3_OFF SEG_D4_OFF
}

void digital_2_off(void)
{
	SEG_A_OFF SEG_B_OFF SEG_D_OFF SEG_E_OFF SEG_G_OFF
}

void digital_3_on(void)
{
	SEG_A_ON SEG_B_ON SEG_C_ON SEG_D_ON SEG_E_OFF SEG_F_OFF SEG_G_ON SEG_D3_OFF SEG_D4_OFF
}

void digital_3_off(void)
{
	SEG_A_OFF SEG_B_OFF SEG_C_OFF SEG_D_OFF SEG_G_OFF
}

void digital_4_on(void)
{
	SEG_A_OFF SEG_B_ON SEG_C_ON SEG_D_OFF SEG_E_OFF SEG_F_ON	SEG_G_ON SEG_D3_OFF SEG_D4_OFF
}

void digital_4_off(void)
{
	SEG_B_OFF SEG_C_OFF SEG_F_OFF	SEG_G_OFF
}

void digital_5_on(void)
{
	SEG_A_ON SEG_B_OFF SEG_C_ON SEG_D_ON SEG_E_OFF SEG_F_ON	SEG_G_ON SEG_D3_OFF SEG_D4_OFF
}

void digital_5_off(void)
{
	SEG_A_OFF SEG_C_OFF SEG_D_OFF SEG_F_OFF	SEG_G_OFF
}

void digital_6_on(void)
{
	SEG_A_ON SEG_B_OFF SEG_C_ON SEG_D_ON SEG_E_ON SEG_F_ON SEG_G_ON SEG_D3_OFF SEG_D4_OFF
}

void digital_6_off(void)
{
	SEG_A_OFF SEG_C_OFF SEG_D_OFF SEG_E_OFF SEG_F_OFF SEG_G_OFF
}

void digital_7_on(void)
{
	SEG_A_ON SEG_B_ON SEG_C_ON SEG_D_OFF SEG_E_OFF SEG_F_OFF SEG_G_OFF SEG_D3_OFF SEG_D4_OFF
}

void digital_7_off(void)
{
	SEG_A_OFF SEG_B_OFF SEG_C_OFF
}

void digital_8_on(void)
{
	SEG_A_ON SEG_B_ON SEG_C_ON SEG_D_ON SEG_E_ON SEG_F_ON SEG_G_ON SEG_D3_OFF SEG_D4_OFF
}

void digital_8_off(void)
{
	SEG_A_OFF SEG_B_OFF SEG_C_OFF SEG_D_OFF SEG_E_OFF SEG_F_OFF SEG_G_OFF
}

void digital_9_on(void)
{
	SEG_A_ON SEG_B_ON SEG_C_ON SEG_D_ON SEG_E_OFF SEG_F_ON SEG_G_ON SEG_D3_OFF SEG_D4_OFF
}

void digital_9_off(void)
{
	SEG_A_OFF SEG_B_OFF SEG_C_OFF SEG_D_OFF SEG_F_OFF SEG_G_OFF
}

static void (*digitals_on []) (void) = {
    digital_0_on,
    digital_1_on,
    digital_2_on,
    digital_3_on,
    digital_4_on,
    digital_5_on,
    digital_6_on,
    digital_7_on,
    digital_8_on,
    digital_9_on
};

static void (*digitals_off []) (void) = {
    digital_0_off,
    digital_1_off,
    digital_2_off,
    digital_3_off,
    digital_4_off,
    digital_5_off,
    digital_6_off,
    digital_7_off,
    digital_8_off,
    digital_9_off
};
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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
    size_t len = sizeof (digitals_on) / sizeof (digitals_on [0]);

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
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_3);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(24000000);
  LL_SetSystemCoreClock(24000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
