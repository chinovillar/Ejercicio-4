/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RETARDO_DEBOUNCE 15 //Definición de retardo para debounce de 15 milisegundos.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t Boton_N;           //Variable para determinar el pulsador que causó la interrupción
bool estado_pulsador=1;     //Booleano que establece el estado predeterminado del pulsador
uint32_t contador=0;

//Mensajes a enviar por UART//
uint8_t mensaje1[20]= "Tecla1: presionada\r\n";
uint8_t mensaje2[20]= "Tecla1: soltada\r\n";
uint8_t mensaje3[20]= "Tecla2: presionada\r\n";
uint8_t mensaje4[20]= "Tecla2: soltada\r\n";


uint8_t byte; //Variable para reicbir mensaje por UART

//comandos que recibie la UART//

typedef enum commandsEnum
{ COMMAND_LED1_TOGGLE = 'a',
COMMAND_LED2_TOGGLE = 's',
COMMAND_LED3_TOGGLE = 'd',
COMMAND_LED4_TOGGLE = 'f'
} commands_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
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
SysTick_Config(SystemCoreClock / 1000);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //Interrupción externa accionada por el pulsador.
{
	Boton_N = GPIO_Pin;      //Asignar a la variable el pin que activó la interrupción.
}

void SysTick_Handler(void)   //Interrupción disparada cada milisegundo.
{


if(Boton_N == 2){ //En caso de que el pin A1 haya sido el que ocasionó la interrupción.
uint8_t estado_actual = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);  //Lectura de estado de entrada


	if(estado_actual != estado_pulsador)   //Función de Debounce
	{
		contador = contador+1;
		if(contador >= RETARDO_DEBOUNCE)
		{

			HAL_UART_Transmit(&huart1, mensaje1, sizeof(mensaje1),100);
			contador= 0;
			estado_pulsador= 1;
			Boton_N=0;
		}

		}
	if (estado_actual == estado_pulsador){ //Función de Debounce

		contador = contador+1;
		if(contador >= RETARDO_DEBOUNCE)
		{
			HAL_UART_Transmit(&huart1, mensaje2, sizeof(mensaje2),100);
			Boton_N=0;
			contador= 0;
		}
	}
	}

if(Boton_N == 8){  //En caso de que el pin A3 haya sido el que ocasionó la interrupción.
uint8_t estado_actual = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);


		if(estado_actual != estado_pulsador) //Función de Debounce
		{
			contador = contador+1;
			if(contador == RETARDO_DEBOUNCE)
			{
				HAL_UART_Transmit(&huart1, mensaje3, sizeof(mensaje3),100);
				contador=0;
				estado_pulsador= 1;
				Boton_N=0;
			}

		}
		if (estado_actual == estado_pulsador){ //Función de Debounce
		contador = contador+1;
		if(contador >= RETARDO_DEBOUNCE)
		{
			HAL_UART_Transmit(&huart1, mensaje4, sizeof(mensaje4),100);
			Boton_N=0;
			contador=0;
		}
}
}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //Interrupción de UART que recibe los comandos
{
	if (byte == COMMAND_LED1_TOGGLE)

	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		HAL_UART_Transmit(&huart1, &byte, sizeof(byte),100); //Transmisión de UART que devuelve lo que recibió para visualizar tecla presionada


	}


	else if (byte == COMMAND_LED2_TOGGLE)

	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
		HAL_UART_Transmit(&huart1, &byte, sizeof(byte),100); //Transmisión de UART que devuelve lo que recibió para visualizar tecla presionada


	}

	else if (byte == COMMAND_LED3_TOGGLE)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
		HAL_UART_Transmit(&huart1, &byte, sizeof(byte),100); //Transmisión de UART que devuelve lo que recibió para visualizar tecla presionada


	}

	else if (byte = COMMAND_LED4_TOGGLE)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		HAL_UART_Transmit(&huart1, &byte, sizeof(byte),100); //Transmisión de UART que devuelve lo que recibió para visualizar tecla presionada


	}

	HAL_UART_Receive_IT(&huart1, &byte, 1); /*Vuelve a habilitar UART para recibir próximo mensaje*/
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
