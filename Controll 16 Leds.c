/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */
	
	//Clock For GPIO
	
	RCC->AHB1ENR |= 1<<0; //GPIOA
	RCC->AHB1ENR |= 1<<1; //GPIOB
	RCC->AHB1ENR |= 1<<2; //GPIOC
	RCC->AHB1ENR |= 1<<3; //GPIOD
	RCC->AHB1ENR |= 1<<4; //GPIOE
	
	//IN PUT ( Button for 16 leds )
	
	//Button P1
	
	GPIOD->MODER |= 0b00<<(11*2); //input D11
	GPIOD->PUPDR |= 1<<(11*2); //pull up
	GPIOD->OSPEEDR |= 1<<(11*2); //medium speed
	
	//Button P2
	
	GPIOC->MODER |= 0b00<<(6*2); //input C6
	GPIOC->PUPDR |= 1<<(6*2); //pull up
	GPIOC->OSPEEDR |= 1<<(6*2); //medium speed
	
	//Button P3
	
	GPIOC->MODER |= 0b00<<(7*2); //input C7
	GPIOC->PUPDR |= 1<<(7*2); //pull up
	GPIOC->OSPEEDR |= 1<<(7*2); //medium speed
	
	//Button P4
	
	GPIOE->MODER |= 0b00<<(14*2); //input C6
	GPIOE->PUPDR |= 1<<(14*2); //pull up
	GPIOE->OSPEEDR |= 1<<(14*2); //medium speed
	
	//OUTPUT ( LEDs )
	
	GPIOA->MODER = 1<<0;
	GPIOA->OSPEEDR = 0<<1;
	GPIOA->OTYPER = 0x0000;
	
	GPIOE->MODER = 1<<0|1<<4*2|1<<5*2|1<<7*2|1<<8*2|1<<9*2|1<<10*2;
	GPIOE->OSPEEDR = 0<<1|0<<4|0<<5|0<<7|0<<8|1<<9|1<<10;
	GPIOE->OTYPER = 0x0000;
	
	GPIOB->MODER = 1<<15*2|1<<14*2|1<<13*2|1<<12*2|1<<10*2|1<<9*2|1<<8*2|1<<7*2;
	GPIOB->OSPEEDR = 0<<15|0<<14|0<<13|0<<12|0<<10|1<<9|1<<8|1<<7;
	GPIOB->OTYPER = 0x0000;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//8 LEDs light from left to right and 8 LEDs light from right to left
		
		if((GPIOD->IDR & (1<<11)) == 0)
		{
			GPIOA->BSRR = (1<<0);
			GPIOB->BSRR = (1<<7);
			HAL_Delay(100);
			GPIOE->BSRR = 1<<10|1<<9|1<<8|1<<7|1<<5|1<<4|1<<16;
			GPIOB->BSRR = 1<<15|1<<14|1<<13|1<<12|1<<10|1<<9|1<<(8+16);
			HAL_Delay(100);
			GPIOE->BSRR = 1<<10|1<<9|1<<8|1<<7|1<<5|1<<(16+4)|1<<0;
			GPIOB->BSRR = 1<<15|1<<14|1<<13|1<<12|1<<10|1<<(16+9)|1<<8;
			HAL_Delay(100);
			GPIOE->BSRR = 1<<10|1<<9|1<<8|1<<7|1<<(16+5)|1<<4|1<<0;
			GPIOB->BSRR = 1<<15|1<<14|1<<13|1<<12|1<<(16+10)|1<<9|1<<8;
			HAL_Delay(100);
			GPIOE->BSRR = 1<<10|1<<9|1<<8|1<<(16+7)|1<<5|1<<4|1<<0;
			GPIOB->BSRR = 1<<15|1<<14|1<<13|1<<(16+12)|1<<10|1<<9|1<<8;
			HAL_Delay(100);
			GPIOE->BSRR = 1<<10|1<<9|1<<(16+8)|1<<7|1<<5|1<<4|1<<0;
			GPIOB->BSRR = 1<<15|1<<14|1<<(16+13)|1<<12|1<<10|1<<9|1<<8;
			HAL_Delay(100);
			GPIOE->BSRR = 1<<10|1<<(16+9)|1<<8|1<<7|1<<5|1<<4|1<<0;
			GPIOB->BSRR = 1<<15|1<<(16+14)|1<<13|1<<12|1<<10|1<<9|1<<8;
			HAL_Delay(100);
			GPIOE->BSRR = 1<<(16+10)|1<<9|1<<8|1<<7|1<<5|1<<4|1<<0;
			GPIOB->BSRR = 1<<(16+15)|1<<14|1<<13|1<<12|1<<10|1<<9|1<<8;
			HAL_Delay(100);
			GPIOE->BSRR = 1<<10;
			GPIOA->BSRR = 1<<16;
			GPIOB->BSRR = 1<<15;
			GPIOB->BSRR = 1<<(7+16);
			HAL_Delay(100);
		}
		
		//16 LEDs Interleaving
		
		if((GPIOC->IDR &(1<<6))==0)
		{
			GPIOA->BSRR = 1<<0;
			GPIOE->BSRR = 1<<4;
			GPIOE->BSRR = 1<<7;
			GPIOE->BSRR = 1<<9;
			GPIOB->BSRR = 1<<15;
			GPIOB->BSRR = 1<<13;
			GPIOB->BSRR = 1<<10;
			GPIOB->BSRR = 1<<8;
			HAL_Delay(100);
			GPIOA->BSRR = 1<<16;
			GPIOE->BSRR = 1<<(16+4);
			GPIOE->BSRR = 1<<(16+7);
			GPIOE->BSRR = 1<<(16+9);
			GPIOB->BSRR = 1<<(16+15);
			GPIOB->BSRR = 1<<(16+13);
			GPIOB->BSRR = 1<<(16+10);
			GPIOB->BSRR = 1<<(16+8);
			HAL_Delay(100);
			GPIOE->BSRR = 1<<0;
			GPIOE->BSRR = 1<<(5);
			GPIOE->BSRR = 1<<(8);
			GPIOE->BSRR = 1<<(10);
			GPIOB->BSRR = 1<<(14);
			GPIOB->BSRR = 1<<(12);
			GPIOB->BSRR = 1<<(9);
			GPIOB->BSRR = 1<<(7);
			HAL_Delay(100);
			GPIOE->BSRR = 1<<16;
			GPIOE->BSRR = 1<<(16+5);
			GPIOE->BSRR = 1<<(16+8);
			GPIOE->BSRR = 1<<(16+10);
			GPIOB->BSRR = 1<<(16+14);
			GPIOB->BSRR = 1<<(16+12);
			GPIOB->BSRR = 1<<(16+9);
			GPIOB->BSRR = 1<<(16+7);
			HAL_Delay(100);
		}
		
		//16 LEDs light from the inside out
		
		if((GPIOC->IDR & (1<<7))==0)
		{
			GPIOE->BSRR = 1<<10;
			GPIOB->BSRR = 1<<15;
			HAL_Delay(200);
			GPIOE->BSRR = 1<<(16+10);
			GPIOB->BSRR = 1<<(16+15);
			GPIOE->BSRR = 1<<9;
			GPIOB->BSRR = 1<<14;
			HAL_Delay(200);
			GPIOE->BSRR = 1<<(16+9);
			GPIOB->BSRR = 1<<(16+14);
			GPIOE->BSRR = 1<<8;
			GPIOB->BSRR = 1<<13;
			HAL_Delay(200);
			GPIOE->BSRR = 1<<(16+8);
			GPIOB->BSRR = 1<<(16+13);
			GPIOE->BSRR = 1<<7;
			GPIOB->BSRR = 1<<12;
			HAL_Delay(200);
			HAL_Delay(200);
			GPIOE->BSRR = 1<<(16+7);
			GPIOB->BSRR = 1<<(16+12);
			GPIOE->BSRR = 1<<5;
			GPIOB->BSRR = 1<<10;
			HAL_Delay(200);
			GPIOE->BSRR = 1<<(16+5);
			GPIOB->BSRR = 1<<(16+10);
			GPIOE->BSRR = 1<<4;
			GPIOB->BSRR = 1<<9;
			HAL_Delay(200);
			GPIOE->BSRR = 1<<(16+4);
			GPIOB->BSRR = 1<<(16+9);
			GPIOE->BSRR = 1<<0;
			GPIOB->BSRR = 1<<8;
			HAL_Delay(200);
			GPIOE->BSRR = 1<<(16+0);
			GPIOB->BSRR = 1<<(16+8);
			GPIOA->BSRR = 1<<0;
			GPIOB->BSRR = 1<<7;
			HAL_Delay(200);
			GPIOA->BSRR = 1<<(16+10);
			GPIOB->BSRR = 1<<(16+7);
			HAL_Delay(200);
		}
		
		//Turn off ALL
		
		if((GPIOE->IDR & (1<<14))==0)
		{
			GPIOA->ODR= 0xFFFF;
			GPIOB->ODR= 0xFFFF;
			GPIOE->ODR= 0xFFFF;
			HAL_Delay(400);
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
