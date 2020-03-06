/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "lwip.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "rs485.h"
#include "eeprom.h"
#include "flash_interface.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	EEPROM_KEY_VALUE	0x3235
#define	CONFIG_KEY_VALUE	0x1215

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern uint16_t rx1_cnt;
extern uint16_t rx1_tmr;
extern uint8_t dir1_tmr;

extern uint16_t rx2_cnt;
extern uint16_t rx2_tmr;
extern uint8_t dir2_tmr;

uint16_t VirtAddVarTab[NB_OF_VAR]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};

uint16_t ai_type = 0xFFFF;
uint8_t ip_addr[4] = {0};
uint8_t ip_mask[4] = {0};
uint8_t ip_gate[4] = {0};

uint16_t rs485_conf1 = 0x1002;
uint16_t rs485_conf2 = 0x1002;

uint8_t baudrate1=2;
uint8_t stop_bits1=1;
uint8_t parity1=0;
uint8_t baud_dir1 = 3;

uint8_t baudrate2=2;
uint8_t stop_bits2=1;
uint8_t parity2=0;
uint8_t baud_dir2 = 3;

extern uint8_t net_address;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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

	uint16_t ee_key = 0;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  //start_application();

  HAL_Delay(1500);
  HAL_FLASH_Unlock();
  EE_Init();
  EE_ReadVariable(VirtAddVarTab[0],  &ee_key);
  if(ee_key!=EEPROM_KEY_VALUE) {
	  EE_WriteVariable(VirtAddVarTab[0],EEPROM_KEY_VALUE);
	  EE_WriteVariable(VirtAddVarTab[1],0); // start program cmd (0-init,1-bootloader finished, 2-correct program)
  }
  EE_ReadVariable(VirtAddVarTab[1],  &ee_key);
  if(ee_key==1) {
	  EE_WriteVariable(VirtAddVarTab[1],0);
	  start_application();
  }else if(ee_key==2) {
	  start_application();
  }
  EE_ReadVariable(VirtAddVarTab[15],  &ee_key);
  if(ee_key!=CONFIG_KEY_VALUE) {
  	  EE_WriteVariable(VirtAddVarTab[0],EEPROM_KEY_VALUE);
  	  EE_WriteVariable(VirtAddVarTab[2],1);			// net address
  	  EE_WriteVariable(VirtAddVarTab[3],0xC0A8);	// IP address
  	  EE_WriteVariable(VirtAddVarTab[4],0x0102);
  	  EE_WriteVariable(VirtAddVarTab[5],0xFFFF);	// IP mask
  	  EE_WriteVariable(VirtAddVarTab[6],0xFF00);
  	  EE_WriteVariable(VirtAddVarTab[7],0xC0A8);	// IP gate
  	  EE_WriteVariable(VirtAddVarTab[8],0x0101);
  	  EE_WriteVariable(VirtAddVarTab[9],0xFFFF);	// ai_type
  	  EE_WriteVariable(VirtAddVarTab[10],0x1002);	// rs485_conf1
  	  EE_WriteVariable(VirtAddVarTab[11],0x1002);	// rs485_conf2
  	  EE_WriteVariable(VirtAddVarTab[15],CONFIG_KEY_VALUE);
  }

  EE_ReadVariable(VirtAddVarTab[2],  &ee_key);
  net_address = ee_key&0xFF;
  EE_ReadVariable(VirtAddVarTab[3],  &ee_key);
  ip_addr[0] = ee_key>>8;
  ip_addr[1] = ee_key&0xFF;
  EE_ReadVariable(VirtAddVarTab[4],  &ee_key);
  ip_addr[2] = ee_key>>8;
  ip_addr[3] = ee_key&0xFF;
  EE_ReadVariable(VirtAddVarTab[5],  &ee_key);
  ip_mask[0] = ee_key>>8;
  ip_mask[1] = ee_key&0xFF;
  EE_ReadVariable(VirtAddVarTab[6],  &ee_key);
  ip_mask[2] = ee_key>>8;
  ip_mask[3] = ee_key&0xFF;
  EE_ReadVariable(VirtAddVarTab[7],  &ee_key);
  ip_gate[0] = ee_key>>8;
  ip_gate[1] = ee_key&0xFF;
  EE_ReadVariable(VirtAddVarTab[8],  &ee_key);
  ip_gate[2] = ee_key>>8;
  ip_gate[3] = ee_key&0xFF;
  EE_ReadVariable(VirtAddVarTab[9],  &ai_type);
  EE_ReadVariable(VirtAddVarTab[10],  &rs485_conf1);
  EE_ReadVariable(VirtAddVarTab[11],  &rs485_conf2);

  baudrate1 = rs485_conf1 & 0xFF;
    baudrate2 = rs485_conf2 & 0xFF;
    parity1 = rs485_conf1 >> 12;
    parity2 = rs485_conf2 >> 12;
    stop_bits1 = (rs485_conf1 >> 8) & 0x0F;
    stop_bits2 = (rs485_conf2 >> 8) & 0x0F;

    switch(baudrate1) {
		case 0:baud_dir1 = 10;break;
		case 1:baud_dir1 = 6;break;
		case 2:baud_dir1 = 3;break;
		case 3:baud_dir1 = 2;break;
		case 4:baud_dir1 = 2;break;
		case 5:baud_dir1 = 2;break;
		case 6:baud_dir1 = 1;break;
    }

    switch(baudrate2) {
		case 0:baud_dir2 = 10;break;
		case 1:baud_dir2 = 6;break;
		case 2:baud_dir2 = 3;break;
		case 3:baud_dir2 = 2;break;
		case 4:baud_dir2 = 2;break;
		case 5:baud_dir2 = 2;break;
		case 6:baud_dir2 = 1;break;
   }

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  /* USER CODE BEGIN 2 */



  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_6);
  LL_USART_EnableIT_RXNE(USART2);
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_7);
  LL_USART_EnableIT_RXNE(USART1);

  LL_USART_Enable(USART1);
  LL_USART_Enable(USART2);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  static uint16_t led_tmr=0;

  if(rx1_cnt) {rx1_tmr++;}else rx1_tmr=0;
  if(dir1_tmr) {
	dir1_tmr--;
	if(dir1_tmr==0) {
		HAL_GPIO_WritePin(RS485_DIR1_GPIO_Port,RS485_DIR1_Pin,GPIO_PIN_RESET);
		LL_USART_EnableIT_RXNE(USART1);
	}
  }

  if(rx2_cnt) {rx2_tmr++;}else rx2_tmr=0;
  if(dir2_tmr) {
	  dir2_tmr--;
	  if(dir2_tmr==0) {
		  HAL_GPIO_WritePin(RS485_DIR2_GPIO_Port,RS485_DIR2_Pin,GPIO_PIN_RESET);
		  LL_USART_EnableIT_RXNE(USART2);
	  }
  }

  uart1_scan();
  uart2_scan();
  led_tmr++;if(led_tmr>=100) {
	  led_tmr = 0;
	  HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
