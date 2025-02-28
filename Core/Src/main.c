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
#include <stdio.h>
#include <string.h>
#include "keypad.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "locked.h"
#include "unlocked.h"
#include "ring_buffer.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FW_VERSION "0.1.0"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t keypad_colum_pressed = 0;
uint32_t key_pressed_tick = 0;
uint8_t b1_press_count = 0;

uint8_t pc_rx_data[3];
ring_buffer_t pc_rx_buffer;

uint8_t keypad_rx_data[3];
ring_buffer_t keypad_rx_buffer;

uint8_t internet_rx_data[3];
ring_buffer_t internet_rx_buffer;

extern char state[3] = "Cl";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int system_events_handler(char *event)
{
  if (strcmp(event, "Op") == 0) {
    return 2;
  }
  else if (strcmp(event, "To") == 0) {
    return 1;
  }
  else if (strcmp(event, "Cl") == 0) {
    return 0;
  }
  return -1; // Return a default value if no condition is met
}
void system_state_machine(char *states)
{
  int state = system_events_handler(states);
  switch (state)
  {
    case 0: // Door closed
      //HAL_UART_Transmit(&huart2, (uint8_t *)"Door closed\r\n", 13, 100);
      break;
    case 1: // Door temporarily opened;
      //HAL_UART_Transmit(&huart2, (uint8_t *)"Door temporarily opened\r\n", 25, 100);
      break;
    case 2: // Door permanent opened
       //HAL_UART_Transmit(&huart2, (uint8_t *)"Door opened\r\n", 13, 100);
      break;
    default:
      break;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
if (GPIO_Pin == B1_Pin) {
  int press_type = detect_button_press();
  if (press_type == 1) {
      strcpy(state, "Op");  // Clic simple
      HAL_UART_Transmit(&huart2, (uint8_t *)"Puerta Abierta\r\n", 17, 100);
  } else if (press_type == 2) {
      strcpy(state, "Cl");  // Doble clic
      HAL_UART_Transmit(&huart2, (uint8_t *)"Puerta Cerrada\r\n", 17, 100);
      
  }
 }
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    HAL_UART_Transmit(&huart3, huart->pRxBuffPtr, huart->RxXferSize, 1000);
    HAL_UART_Receive_IT(&huart2, huart->pRxBuffPtr, huart->RxXferSize);
  } else if (huart->Instance == USART3) {
    HAL_UART_Transmit(&huart2, huart->pRxBuffPtr, huart->RxXferSize, 1000);
    HAL_UART_Receive_IT(&huart3, huart->pRxBuffPtr, huart->RxXferSize);
  }
}

/**
 * @brief  Heartbeat function to blink LED2 every 1 second to indicate the system is running
*/
void heartbeat(void)
{
  static uint32_t last_heartbeat = 0;
  if (HAL_GetTick() - last_heartbeat > 1000)
  {
    last_heartbeat = HAL_GetTick();
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
  }
}

/**
 * @brief  Retargets the C library printf function to the USART.
*/
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
  return len;
}

void OLED_Printer(ring_buffer_t *rb, uint8_t *buffer, char *state)
{
  (void)rb;
  if(strcmp((char *)buffer, "#*0*#") == 0 )
  {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Buffer:Clean", Font_7x10, White);
    ssd1306_SetCursor(0, 30);
    ssd1306_WriteString("Door:Cl", Font_7x10, White);
    ssd1306_UpdateScreen();
    HAL_Delay(2000); 
    ssd1306_Fill(Black);
    ssd1306_DrawBitmap(0, 0, locked, 128, 64, White);
    ssd1306_UpdateScreen();
    return;
  }

  if (strcmp((char *)buffer, "#O#") == 0)
  {
    if (strcmp(state, "Op") == 0)
    {
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("Puerta ya esta abierta", Font_7x10, White);
      ssd1306_UpdateScreen();
      HAL_Delay(2000); 
      ssd1306_Fill(Black);
      ssd1306_DrawBitmap(0, 0, unlocked, 128, 64, White);
      ssd1306_UpdateScreen();
    }
    else
    {
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("Door: Op", Font_7x10, White);
      ssd1306_UpdateScreen();
      HAL_Delay(2000); 
      ssd1306_Fill(Black);
      ssd1306_DrawBitmap(0, 0, unlocked, 128, 64, White);
      ssd1306_UpdateScreen();
   }
  }

 else if (strcmp((char *)buffer, "#C#") == 0)
 {
  if (strcmp(state, "Cl") == 0)
  {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Puerta ya esta cerrada", Font_7x10, White);
    ssd1306_UpdateScreen();
    HAL_Delay(2000); 
    ssd1306_Fill(Black);
    ssd1306_DrawBitmap(0, 0, unlocked, 128, 64, White);
    ssd1306_UpdateScreen();
  }
  else
  {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Door: Cl", Font_7x10, White);
    ssd1306_UpdateScreen();
    HAL_Delay(2000); 
    ssd1306_Fill(Black);
    ssd1306_DrawBitmap(0, 0, unlocked, 128, 64, White);
    ssd1306_UpdateScreen();
  } 
 }
}

void process_command(ring_buffer_t *rb, uint8_t *buffer, char *state) {
  if (ring_buffer_size(rb) == 3) {  // Verifica si el comando es de longitud 5
      // Lee el comando completo del buffer
      for (int i = 0; i < 3; i++) {
          ring_buffer_read(rb, &buffer[i]);
      }
      buffer[3] = '\0';  // Asegura el término del string

      // Procesa el comando basado en su contenido
      if (strcmp((char *)buffer, "#O#") == 0) {
          if (strcmp(state, "Op") == 0) {
              HAL_UART_Transmit(&huart2, (uint8_t *)"Puerta ya esta abierta\r\n", 26, 100);
              OLED_Printer(rb, buffer, state);
          } else {
              HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);  // Cambia el estado del LED
              HAL_UART_Transmit(&huart2, (uint8_t *)"Puerta Abierta\r\n", 17, 100);
              OLED_Printer(rb, buffer, state);
              strcpy(state, "Op");
          }

      } 

      else if (strcmp((char *)buffer, "#C#") == 0) {
          if (strcmp(state, "Cl") == 0) {
              HAL_UART_Transmit(&huart2, (uint8_t *)"Puerta ya esta cerrada\r\n", 27, 100);
              OLED_Printer(rb, buffer, state);
          } else {
              HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);  // Cambia el estado del LED
              HAL_UART_Transmit(&huart2, (uint8_t *)"Puerta Cerrada\r\n", 17, 100);
              OLED_Printer(rb, buffer, state);
              strcpy(state, "Cl");
          }
      } 

      else if (strcmp((char *)buffer, "#0#") == 0) {
          HAL_UART_Transmit(&huart2, (uint8_t *)"Buffer limpiado y puerta cerrada\r\n", 38, 100);
          ring_buffer_reset(rb);
          OLED_Printer(rb, buffer, state);
          strcpy(state, "Cl");
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          HAL_Delay(500);
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          HAL_Delay(500);
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          HAL_Delay(500);
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      } 

      else {
          HAL_UART_Transmit(&huart2, (uint8_t *)"Comando no reconocido\r\n", 24, 100);
          if (strcmp(state, "Cl") == 0) {
              ssd1306_Fill(Black);
              ssd1306_SetCursor(0, 0);
              ssd1306_WriteString((char *)"Comand unknown", Font_7x10, White);
              ssd1306_UpdateScreen();
              HAL_Delay(2000); 
              ssd1306_Fill(Black);
              ssd1306_DrawBitmap(0, 0, locked, 128, 64, White);
              ssd1306_UpdateScreen();
          } else {
              ssd1306_Fill(Black);
              ssd1306_SetCursor(0, 0);
              ssd1306_WriteString((char *)"Comand unknown", Font_7x10, White);
              ssd1306_UpdateScreen();
              HAL_Delay(2000); 
              ssd1306_Fill(Black);
              ssd1306_DrawBitmap(0, 0, unlocked, 128, 64, White);
              ssd1306_UpdateScreen();
          }
      }

      // Reinicia el buffer de comando
      for (int i = 0; i < 3; i++) {
          buffer[i] = '_';
      }
  }
}
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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Receive_IT(&huart2, pc_rx_data, sizeof(pc_rx_data));
  //HAL_UART_Receive_IT(&huart3, keypad_rx_data, sizeof(keypad_rx_data));
  
  ring_buffer_init(&pc_rx_buffer, pc_rx_data, sizeof(pc_rx_data));
  ring_buffer_init(&keypad_rx_buffer, keypad_rx_data, sizeof(keypad_rx_data));
  ring_buffer_init(&internet_rx_buffer, internet_rx_data, sizeof(internet_rx_data));
  setvbuf(stdout, NULL, _IONBF, 0);  // Desactiva el buffer de stdout

  keypad_init();
  HAL_UART_Transmit(&huart2, (uint8_t *)"Hello World\r\n\0", 20, 100);  // Envía el mensaje "Hello World"
  ssd1306_WriteString("Door default: Cl", Font_7x10, White);
  HAL_Delay(2000); //
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_DrawBitmap(0, 0, locked, 128, 64, White);
  ssd1306_UpdateScreen();
  uint8_t pc_key;
  uint8_t key;

  for (size_t i = 0; i < sizeof(keypad_rx_data); i++) {
    keypad_rx_data[i] = '_';
    pc_rx_data[i] = '_';
  } 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    heartbeat();
    if (keypad_colum_pressed != 0 && (key_pressed_tick + 5) < HAL_GetTick()) {
      key = keypad_scan(keypad_colum_pressed);
      keypad_colum_pressed = 0;
      if (key != 'E') {
        ring_buffer_write(&keypad_rx_buffer, key);
        uint8_t size = ring_buffer_size(&keypad_rx_buffer);
        char msg[45];
        snprintf(msg, sizeof(msg), "Key: %c, Buffer: %s, Size: %d\r\n", key, keypad_rx_data, size);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
      }
      // Procesa teclas recibidas desde la PC
    }
    if (HAL_UART_Receive(&huart2, &pc_key, 1, 10) == HAL_OK) {
      ring_buffer_write(&pc_rx_buffer, pc_key);
      uint8_t size = ring_buffer_size(&pc_rx_buffer);
      char msg[45];
      snprintf(msg, sizeof(msg), "PC Key: %c, Buffer: %s, Size: %d\r\n", pc_key, pc_rx_data, size);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    }
    system_events_handler(key);
    process_command(&keypad_rx_buffer, keypad_rx_data, state);
    process_command(&pc_rx_buffer, pc_rx_data, state);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    system_state_machine((char *)state);
    HAL_Delay(100);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, DOOR_Pin|LD2_Pin|LD1_Pin|ROW_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ROW_2_Pin|ROW_4_Pin|ROW_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RING_Pin */
  GPIO_InitStruct.Pin = RING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RING_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DOOR_Pin LD2_Pin LD1_Pin */
  GPIO_InitStruct.Pin = DOOR_Pin|LD2_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : COLUMN_1_Pin */
  GPIO_InitStruct.Pin = COLUMN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COLUMN_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COLUMN_4_Pin */
  GPIO_InitStruct.Pin = COLUMN_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COLUMN_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COLUMN_2_Pin COLUMN_3_Pin */
  GPIO_InitStruct.Pin = COLUMN_2_Pin|COLUMN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW_1_Pin */
  GPIO_InitStruct.Pin = ROW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ROW_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW_2_Pin ROW_4_Pin ROW_3_Pin */
  GPIO_InitStruct.Pin = ROW_2_Pin|ROW_4_Pin|ROW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
