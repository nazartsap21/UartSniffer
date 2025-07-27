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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_MESSAGES 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
void Task1_App(void *argument);
void Task2_App(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t  uart1_rx_buffer[512] = {0};
uint8_t  uart1_rx_byte        = 0;
uint16_t uart1_byte_received  = 0;

uint8_t  uart3_rx_buffer[512] = {0};
uint8_t  uart3_rx_byte        = 0;
uint16_t uart3_byte_received  = 0;

uint8_t  transmit_buffer[]    = "Hello from STM32!\n\r";


uint8_t  btn_state;
uint8_t  prev_btn_state;

uint8_t  receive;


typedef enum
{
  MESSAGE_END_BYTE = 0,
  MESSAGE_END_NEWLINE,
  MESSAGE_END_CURSOR,
  MESSAGE_END_NEWLINE_CURSOR,

  MESSAGE_END_TOTAL
} message_end_condition;


typedef enum
{
  SNIFFER_BUFFER_1 = 0,
  SNIFFER_BUFFER_2,

  SNIFFER_BUFFER_TOTAL
} buffer_id;


typedef struct {
  uint8_t  message[64];
  uint8_t  length;
  uint8_t  line;
  uint32_t timestamp;
} received_data_t;


received_data_t       received_data[MAX_MESSAGES] = {0};
uint8_t               message_index     = 0;
uint8_t               keep_receiving    = 0;

buffer_id             uart1_rx_id       = SNIFFER_BUFFER_1;
buffer_id             uart3_rx_id       = SNIFFER_BUFFER_2;
message_end_condition message_end_state = MESSAGE_END_CURSOR;


void clear_buffer(uint8_t buffer_id)
{
  switch (buffer_id)
  {
    case SNIFFER_BUFFER_1:
      memset(uart1_rx_buffer, 0, sizeof(uart1_rx_buffer));
      break;
    case SNIFFER_BUFFER_2:
      memset(uart3_rx_buffer, 0, sizeof(uart3_rx_buffer));
      break;
    default:
      return;
  }
}


void clear_byte(uint8_t buffer_id)
{
  switch (buffer_id)
  {
    case SNIFFER_BUFFER_1:
      memset(&uart1_rx_byte, 0, sizeof(uart1_rx_byte));
      break;
    case SNIFFER_BUFFER_2:
      memset(&uart3_rx_byte, 0, sizeof(uart3_rx_byte));
      break;
    default:
      return;
  }
}


void print_all_received_messages(void)
{
  for (uint8_t i = 0; i < message_index; i++)
  {
    char timebuf[32];
    char linebuf[16];
    sprintf(timebuf, "[%lu ms] ", received_data[i].timestamp);
    HAL_UART_Transmit(&huart2, (uint8_t *)timebuf, strlen(timebuf), 100);
    sprintf(linebuf, "UART%d: ", received_data[i].line == SNIFFER_BUFFER_1 ? 1 : 3);
    HAL_UART_Transmit(&huart2, (uint8_t *)linebuf, strlen(linebuf), 100);
    HAL_UART_Transmit(&huart2, received_data[i].message, received_data[i].length, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
  }

  message_index = 0;
}


void handle_receive(uint8_t buffer_byte, uint8_t buffer_id)
{
  static uint8_t  temp_buffer[64];
  static uint16_t temp_index = 0;

  if (message_end_state >= MESSAGE_END_TOTAL)
    return;

  temp_buffer[temp_index++] = buffer_byte;

  if (temp_index >= 64) {
    temp_index = 0;
    return;
  }

  uint8_t is_end = 0;
  switch (message_end_state)
  {
    case MESSAGE_END_BYTE:
      is_end = 1;
      break;
    case MESSAGE_END_NEWLINE:
      if (buffer_byte == '\n')
        is_end = 1;
      break;
    case MESSAGE_END_CURSOR:
      if (buffer_byte == '\r')
        is_end = 1;
      break;
    case MESSAGE_END_NEWLINE_CURSOR:
      if (buffer_byte == '\r' && temp_index > 1 && temp_buffer[temp_index - 2] == '\n')
        is_end = 1;
      break;
    default:
      break;
  }

  if (is_end) {
    memcpy(received_data[message_index].message, temp_buffer, temp_index);
    received_data[message_index].length    = temp_index;
    received_data[message_index].line      = buffer_id;
    received_data[message_index].timestamp = HAL_GetTick();
    if (++message_index >= MAX_MESSAGES)
    {
      keep_receiving = 0;
      receive        = 0;
      HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Sniffing OFF\r\n", 14);
      print_all_received_messages();
    }

    temp_index = 0;
    memset(temp_buffer, 0, sizeof(temp_buffer));
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (!receive)
  {
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Sniffing stopped\r\n", 18);
    return;
  }

  if (huart == &huart1)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    handle_receive(uart1_rx_byte, uart1_rx_id);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    if (keep_receiving)
    {
      HAL_UART_Receive_DMA(&huart1, &uart1_rx_byte, 1);
    }
  }
  if (huart == &huart3)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    handle_receive(uart3_rx_byte, uart3_rx_id);\
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    if (keep_receiving)
    {
      HAL_UART_Receive_DMA(&huart3, &uart3_rx_byte, 1);
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1 */
  Task1Handle = osThreadNew(Task1_App, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(Task2_App, NULL, &Task2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//    HAL_UART_Transmit(&huart2, transmit_buffer, sizeof(transmit_buffer)/sizeof(transmit_buffer[0]), 100);
//    HAL_Delay(2000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.Mode = UART_MODE_RX;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task1_App */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task1_App */
void Task1_App(void *argument)
{
  /* USER CODE BEGIN 5 */
  btn_state = 1;
  prev_btn_state = 1;
  receive = 0;
  /* Infinite loop */
  for(;;)
  {
    btn_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    if (!btn_state)
    {
      prev_btn_state = btn_state;
    }
    else if (btn_state && !prev_btn_state)
    {
      receive        ^= 1;
      keep_receiving ^= 1;
      if (receive)
      {
        clear_buffer(SNIFFER_BUFFER_1);
        clear_buffer(SNIFFER_BUFFER_2);
        osDelay(1);
        HAL_UART_Receive_DMA(&huart1, &uart1_rx_byte, 1);
        HAL_UART_Receive_DMA(&huart3, &uart3_rx_byte, 1);
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Sniffing ON\r\n", 13);
      }
      else
      {
        HAL_UART_DMAStop(&huart1);
        HAL_UART_DMAStop(&huart3);
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Sniffing OFF\r\n", 14);
        print_all_received_messages();
      }

      prev_btn_state = btn_state;
    }
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task2_App */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task2_App */
void Task2_App(void *argument)
{
  /* USER CODE BEGIN Task2_App */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END Task2_App */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
