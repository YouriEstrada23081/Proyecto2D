/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define diviceAddress 0x55
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SPIBUFFER 6
#define I2CBUFFER 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t spi_tx_buf[SPIBUFFER];
uint8_t spi_rx_buf[SPIBUFFER];
uint8_t i2c_rx_buf[I2CBUFFER];

char uart_msg[100];

uint8_t Menu = 0;

uint8_t led_num = 0;
int32_t time_ms = 0;

HAL_StatusTypeDef i2c_status;
int32_t pot_valor = 0;
uint8_t i2c_tx_cmd = 2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void transmit_uart(char *string){
	uint16_t len = strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t*) string, len, 1000);
}

void leerM(char* buffer, uint16_t max_len);
void vaciarBuffer(void);

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Menu = 0; //variable del menu
	  led_num = 0; //variable para los leds
	  time_ms = 0; //variable para el tiempo
	  //imprimir menu
	  transmit_uart("\r\n--- Menu Principal ---\r\n");
	  transmit_uart("1. Control de LEDS (SPI)\r\n");
	  transmit_uart("2. Obtener sensor (I2C)\r\n");
	  transmit_uart("Seleccione una opcion (1 o 2): \r\n");



	  //habilitar siempre la interrupcion esta es bloqueante
	  HAL_UART_Receive(&huart2, &Menu, 1, HAL_MAX_DELAY);
	  //siempre tener la lectura del esp para evitar variaciones aunque esta se valida en la opcion 2
	  i2c_status = HAL_I2C_Master_Receive(&hi2c1, (diviceAddress << 1) | 0x01, i2c_rx_buf, I2CBUFFER, 100);
	  if(Menu == '1') //si se elige 1
	  		{
	  			transmit_uart("Opcion 1: SPI. \r\n");
	  			transmit_uart("Ingrese comando (Ej: 1,500 para LED 1, 500ms): \r\n");
	  			//se espera el mensaje de led_num, time_ms
	  			char input_buffer[20]; //mensaje a leer
	  			int mensajeled = 0;

	  			leerM(input_buffer, 20); //funcion para leer la entrada del usario

	  			int datos = sscanf(input_buffer, "%d,%ld", &mensajeled, &time_ms); //parte el mensaje los mensajes en el tiempo y las leds

	  			if (datos != 2) { //si el mensaje no cuenta con 2 numeros como se pide pues muestra el siguiente error
	  				transmit_uart("Formato incorrecto. Volviendo al menu.\r\n");
	  				continue;
	  			}

	  			led_num = (uint8_t)mensajeled; //convierte el led a un dato de 8 bits
	  			if (led_num < 1 || led_num > 3) { //si el numero no esta dentro del rango muestra el error
	  				 transmit_uart("Numero de LED invalido (debe ser 1, 2, o 3).\r\n");
	  				 continue;
	  			}
	  			if (time_ms < 0 || time_ms > 9999) { //si el numero no esta dentro del rango muestra el error
	  				 transmit_uart("Tiempo invalido.\r\n");
	  				 continue;
	  			}

	  			memset(spi_tx_buf, 0, SPIBUFFER); // vaciar el buffer
	  			spi_tx_buf[0] = 1; //comando para el esp
	  			spi_tx_buf[1] = led_num; //siguiente dato a guardar el de las leds
	  			//se parte el mensaje del tiempo en 4 bytes
	  			spi_tx_buf[2] = (uint8_t)(time_ms >> 24); // al tener el tiempo en 32 bits necesario aclarar que bits a enviar
	  			spi_tx_buf[3] = (uint8_t)(time_ms >> 16);
	  			spi_tx_buf[4] = (uint8_t)(time_ms >> 8);
	  			spi_tx_buf[5] = (uint8_t)(time_ms & 0xFF);
	  			//imprimir que se manda al esp
	  			sprintf(uart_msg, "Enviando: LED=%d, Tiempo=%ldms...\r\n", led_num, time_ms);
	  			transmit_uart(uart_msg);
	  			//comunicacion SPI
	  			HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_RESET); //activa el chip select
	  			HAL_SPI_TransmitReceive(&hspi1, spi_tx_buf, spi_rx_buf, SPIBUFFER, HAL_MAX_DELAY); //envia el mensaje y recibe una respuesta
	  			HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET); //desactiva el chip select
	  			//respuesta recibida del esp
	  			sprintf(uart_msg, "Comando enviado. Respuesta del ESP32: %d\r\n", spi_rx_buf[0]);
	  			transmit_uart(uart_msg);
	  			HAL_Delay(time_ms); //congelar el programa hasta que se acabe el encendido del led
	  		}
	  		else if(Menu == '2') //si se presiono la tecla 2
	  		{
	  			transmit_uart("Opcion 2: I2C. Pidiendo datos al ESP32...\r\n");
	  			//envia un mensaje para solicitar el valor del potenciometro
	  			i2c_status = HAL_I2C_Master_Transmit(&hi2c1, (diviceAddress << 1), &i2c_tx_cmd, 1, 100);

	  			if (i2c_status != HAL_OK) //si no se logro mandar el mensaje mostrar el error
	  			{
	  				sprintf(uart_msg, "Error al enviar comando. Status: %d\r\n", i2c_status);
	  				transmit_uart(uart_msg);
	  			}
	  			else
	  			{
	  				HAL_Delay(20); //pequeña espera para esperar al esclavo

	  				//limpiar el buffer
	  				memset(i2c_rx_buf, 0, I2CBUFFER);

	  				if (i2c_status != HAL_OK) //si no se consigue recibir el mensaje mostrar error
	  				{
	  					transmit_uart("Error al recibir los 4 bytes.\r\n");
	  				}
	  				else
	  				{
	  					i2c_status = HAL_I2C_Master_Receive(&hi2c1, (diviceAddress << 1) | 0x01, i2c_rx_buf, I2CBUFFER, 100);
	  					pot_valor = 0;
	  					pot_valor |= ((int32_t)i2c_rx_buf[0] << 0); //parecido al tiempo se guarda el mensaje reconstruyendo el mensaje
	  					pot_valor |= ((int32_t)i2c_rx_buf[1] << 8);
	  					pot_valor |= ((int32_t)i2c_rx_buf[2] << 16);
	  					pot_valor |= ((int32_t)i2c_rx_buf[3] << 24);
	  					//mostrar el valor
	  					sprintf(uart_msg, "Valor del Potenciometro (PT1) = %ld\r\n", pot_valor);
	  					transmit_uart(uart_msg);
	  				}
	  			}
	  			vaciarBuffer(); //limpia buffer
	  		}
	  		else
	  		{
	  			transmit_uart("Opcion no valida. Intente de nuevo.\r\n");
	  			vaciarBuffer();
	  		}

	  		HAL_Delay(200);



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP32_CS_Pin */
  GPIO_InitStruct.Pin = ESP32_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(ESP32_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void leerM(char* buffer, uint16_t max_len) {
	uint16_t index = 0; //posicion para escirbir
	uint8_t received_char; //mensaje recibido
	memset(buffer, 0, max_len); //limpiar el buffer antes de comprobar

	while (index < (max_len - 1)) { //seguir leyendo mientras que tenga espacio el buffer siempre se guarad un espacio para el ccomando del esp
		if (HAL_UART_Receive(&huart2, &received_char, 1, HAL_MAX_DELAY) != HAL_OK) { //si hay un error sale de la funcion
			return;
		}
		// comprueba si el mensaje se termino por medio de enter o espacio
		if (received_char == '\r' || received_char == '\n') {
			if (index > 0) {
				//si ya se recibieron datos y se encuentra el enter sale de la funcion
				break;
			}
			else { //si no se ha recibido datos pues seguir aqui
				continue;
			}

		}
		else if (received_char >= ' ' && received_char <= '~') { //solo se aceptan valores ASCII imprimibles
			buffer[index++] = received_char; //añade el caracter al buffer y sumamos al index
		}

	}

	buffer[index] = '\0'; //esto se añade al final para que la funcion de sscanf sepa donde terminar la cadena de texto
}

void vaciarBuffer(void) {
	uint8_t temp_char; //variable para guardar los bytes leids
	while (HAL_UART_Receive(&huart2, &temp_char, 1, 1) == HAL_OK) {
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
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
