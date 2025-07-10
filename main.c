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

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "SEGGER_EX.h"
#include "queue.h"
#include "string.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rtc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DWT_CTRL								(*(volatile uint32_t*)0xE0001000)

#define state_rtc_set_time_sec						0
#define state_rtc_set_time_min						1
#define state_rtc_set_time_hour_format				2
#define state_rtc_set_time_hour_12					3
#define state_rtc_set_time_hour_24					4
#define state_main_menu								5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

	TaskHandle_t menu_handle;
	TaskHandle_t led_handle;
	TaskHandle_t print_handle;
	TaskHandle_t cmd_handle;
	TaskHandle_t rtc_handle;

	UART_HandleTypeDef husart3;

	RTC_HandleTypeDef hrtc;
	RTC_TimeTypeDef t_hrtc_time;
	RTC_DateTypeDef t_hrtc_date;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

static void MENU_handler(void* parameters);
static void LED_handler(void* parameters);
static void PRINT_handler(void* parameters);
static void CMD_handler(void* parameters);
static void RTC_handler(void* parameters);
static void format_useroption(char *msg, int len);
static void get_curr_time(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct{
	char* msg_item;
	int len;
}menu_item;

QueueHandle_t qprint = NULL;
QueueHandle_t quser_cmd = NULL;
QueueHandle_t qtime_set = NULL;
QueueHandle_t qdate_set = NULL;

char main_menu[] = 	"!!!Welcome to our Show!!!\n"
					"Select one from below:\n"
					"1 -> LED_SHOW\n"
					"2 -> TIME_SHOW\n"
					"*****************\n"
					"***Option:***\n"
					"*****************\n";

char task1_menu[] =	"!!!Welcome to LED Show!!!\n"
					"Select one led effect from below:\n"
					"e1\n"
					"e2\n"
					"e3\n"
					"none\n";

char task2_menu[] =	"!!!Welcome to TIME Show!!!\n"
					"Select one option from below:\n"
					"o1 -> Get Current Time_Date\n"
					"o2 -> Set Time\n"
					"o3 -> Set Date\n"
					"Current Time: ";

char task2_menu_sec[] = "Enter Seconds(0 - 59): \n";
char task2_menu_min[] = "Enter Minutes(0 - 59): \n";
char task2_menu_Hour_Format[] = "Enter Hour Format\n 0 -> 12 Hour\n 1 -> 24 Hour: \n";
char task2_menu_hours_12[] = "Enter Hours(0 - 12): \n";
char task2_menu_hours_24[] = "Enter Hours(0 - 24): \n";

menu_item task2_set_time_menu[5] = {0};

menu_item* task2_set_time_menu_ptr[] = {&task2_set_time_menu[0], &task2_set_time_menu[1],
										&task2_set_time_menu[2], &task2_set_time_menu[3],
								        &task2_set_time_menu[4]};

char user_msg[10] = {0};

char err_msg[] = "Invalid Number!!!\n Please select again:\n\n";

uint8_t err_msg_len = 41;

menu_item t_err_msg = {0};

menu_item* t_err_msg_ptr = &t_err_msg;

char uart_receive = {0};

char current_time[30] = {0};
char current_date[40] = {0};
char current_time_date[100] = {0};
char print_time[200] = {0};

int curr_state;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	qprint = xQueueCreate(1, sizeof(menu_item*));
	quser_cmd = xQueueCreate(10, sizeof(menu_item*));
	qtime_set = xQueueCreate(5, sizeof(uint8_t));
	qdate_set = xQueueCreate(5, sizeof(uint8_t));

	err_msg_len = strlen(err_msg);

	BaseType_t status1;
	BaseType_t status2;
	BaseType_t status3;
	BaseType_t status4;
	BaseType_t status5;

	t_err_msg.msg_item = err_msg;
	t_err_msg.len = err_msg_len;

	task2_set_time_menu[0].msg_item = task2_menu_sec;
	task2_set_time_menu[0].len = strlen(task2_menu_sec);

	task2_set_time_menu[1].msg_item = task2_menu_min;
	task2_set_time_menu[1].len = strlen(task2_menu_min);

	task2_set_time_menu[2].msg_item = task2_menu_Hour_Format;
	task2_set_time_menu[2].len = strlen(task2_menu_Hour_Format);

	task2_set_time_menu[3].msg_item = task2_menu_hours_12;
	task2_set_time_menu[3].len = strlen(task2_menu_hours_12);

	task2_set_time_menu[4].msg_item = task2_menu_hours_24;
	task2_set_time_menu[4].len = strlen(task2_menu_hours_24);


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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  //Enable the Cycle Counter
  traceSTART();

  status1 = xTaskCreate(MENU_handler, "MENU", 200, NULL, 4, &menu_handle);

  configASSERT(status1 == pdPASS);

  status2 = xTaskCreate(LED_handler, "LED", 200, NULL, 2, &led_handle);

  configASSERT(status2 == pdPASS);

  status3 = xTaskCreate(PRINT_handler, "PRINT", 200, NULL, 2, &print_handle);

  configASSERT(status3 == pdPASS);

  status4 = xTaskCreate(CMD_handler, "CMD", 1024, NULL, 2, &cmd_handle);

  configASSERT(status4 == pdPASS);

  status5 = xTaskCreate(RTC_handler, "RTC", 1024, NULL, 2, &rtc_handle);

  configASSERT(status5 == pdPASS);

  //start scheduler
  vTaskStartScheduler();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

  //USART3 Initialize

  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_USART_ENABLE(&husart3);

//  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


  husart3.Instance = USART3;
  husart3.Init.Mode = USART_MODE_TX_RX;
  husart3.Init.StopBits = USART_STOPBITS_1;
  husart3.Init.Parity = USART_PARITY_NONE;
  husart3.Init.BaudRate = 115200;
  husart3.Init.WordLength = USART_WORDLENGTH_8B;
  husart3.Init.OverSampling = UART_OVERSAMPLING_16;
  husart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;

  HAL_UART_Init(&husart3);

  HAL_NVIC_SetPriority(USART3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);



/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//ISRs


static void MENU_handler(void* parameters)
{
	while(1)
	{
		menu_item qmsgs = {main_menu, strlen(main_menu)};

		menu_item* qmsgs_ptr = &qmsgs;

		if(xQueueSend(qprint, &qmsgs_ptr, portMAX_DELAY) == pdPASS)
		{
			xTaskNotify(print_handle, 0, eNoAction);
		}

		while(xTaskNotifyWait(0, 0, 0, portMAX_DELAY) == pdFAIL);

	}
}

static void CMD_handler(void* parameters)
{
	while(1)
	{

		while(xTaskNotifyWait(0, 0, 0, portMAX_DELAY) == pdFAIL);

		format_useroption(user_msg, strlen(user_msg));


		if(strcmp(user_msg, "#") == 0)
		{
			xTaskNotify(menu_handle, 0, eNoAction);
		}
		else if(strcmp(user_msg, "1") == 0)
		{
			xTaskNotify(led_handle, 0, eNoAction);

			while(1)
			{

				while(xTaskNotifyWait(0, 0, 0, portMAX_DELAY) == pdFAIL);

				format_useroption(user_msg, strlen(user_msg));

				if(strcmp(user_msg, "#") == 0)
				{
					xTaskNotify(led_handle, '#', eSetValueWithOverwrite);
					xTaskNotify(menu_handle, 0, eNoAction);
					curr_state = state_main_menu;
					break;
				}
				else if(strcmp(user_msg, "e1") == 0)
				{
					xTaskNotify(led_handle, 1, eSetValueWithoutOverwrite);
				}
				else if(strcmp(user_msg, "e2") == 0)
				{
					xTaskNotify(led_handle, 2, eSetValueWithoutOverwrite);
				}
				else if(strcmp(user_msg, "e3") == 0)
				{
					xTaskNotify(led_handle, 3, eSetValueWithoutOverwrite);
				}
			}
		}
		else if(strcmp(user_msg, "2") == 0)
		{
			xTaskNotify(rtc_handle, 0, eNoAction);

			while(1)
			{
				while(xTaskNotifyWait(0, 0, 0, portMAX_DELAY) == pdFAIL);

				format_useroption(user_msg, strlen(user_msg));

				if(strcmp(user_msg, "#") == 0)
				{
					xTaskNotify(rtc_handle, '#', eSetValueWithOverwrite);
					xTaskNotify(menu_handle, 0, eNoAction);
					break;
				}
				else if(strcmp(user_msg, "o1") == 0)
				{
					xTaskNotify(rtc_handle, 1, eSetValueWithOverwrite);
				}
				else if(strcmp(user_msg, "o2") == 0)
				{
					xTaskNotify(rtc_handle, 2, eSetValueWithOverwrite);

					while(1)
					{
						xTaskNotifyWait(0, 0, 0, portMAX_DELAY);

						format_useroption(user_msg, strlen(user_msg));

						if(strcmp(user_msg, "#") == 0)
						{
							xTaskNotify(rtc_handle, 2, eSetValueWithOverwrite);
							xTaskNotify(menu_handle, 0, eNoAction);
							curr_state = state_main_menu;
							break;
						}
						else if(curr_state == state_rtc_set_time_sec)
						{
							int sec = 0;
							sscanf(user_msg, "%d", &sec);
							if(sec >= 0 && sec < 60)
							{
								xQueueSend(qtime_set, &sec, portMAX_DELAY);
								xTaskNotify(rtc_handle, 1, eSetValueWithOverwrite);
							}
							else
							{
								xQueueSend(qprint, &t_err_msg_ptr, portMAX_DELAY);
								xTaskNotify(print_handle, 0, eNoAction);
								vTaskDelay(pdMS_TO_TICKS(100));
								xTaskNotify(rtc_handle, 0, eSetValueWithOverwrite);
								continue;
							}
						}
						else if(curr_state == state_rtc_set_time_min)
						{
							int min = 0;
							sscanf(user_msg, "%d", &min);
							if(min >= 0 && min < 60)
							{
								xQueueSend(qtime_set, &min, portMAX_DELAY);
								xTaskNotify(rtc_handle, 1, eSetValueWithOverwrite);
							}
							else
							{
								xQueueSend(qprint, &t_err_msg_ptr, portMAX_DELAY);
								xTaskNotify(print_handle, 0, eNoAction);
								vTaskDelay(pdMS_TO_TICKS(100));
								xTaskNotify(rtc_handle, 0, eSetValueWithOverwrite);
								continue;
							}
						}
						else if(curr_state == state_rtc_set_time_hour_format)
						{
							int hour_format = 0;
							sscanf(user_msg, "%d", &hour_format);
							if(hour_format == 0)
							{
								xQueueSend(qtime_set, &hour_format, portMAX_DELAY);
								xTaskNotify(rtc_handle, 3, eSetValueWithOverwrite);
							}
							else if(hour_format == 1)
							{
								xQueueSend(qtime_set, &hour_format, portMAX_DELAY);
								xTaskNotify(rtc_handle, 4, eSetValueWithOverwrite);
							}
							else
							{
								xQueueSend(qprint, &t_err_msg_ptr, portMAX_DELAY);
								xTaskNotify(print_handle, 0, eNoAction);
								vTaskDelay(pdMS_TO_TICKS(100));
								xTaskNotify(rtc_handle, 0, eSetValueWithOverwrite);
								continue;
							}
						}
						else if(curr_state == state_rtc_set_time_hour_12)
						{
							int hour = 0;
							sscanf(user_msg, "%d", &hour);
							if(hour >= 0 && hour < 12)
							{
								xQueueSend(qtime_set, &hour, portMAX_DELAY);
								xTaskNotify(rtc_handle, 2, eSetValueWithOverwrite);
								curr_state = state_main_menu;
								break;
							}
							else
							{
								xQueueSend(qprint, &t_err_msg_ptr, portMAX_DELAY);
								xTaskNotify(print_handle, 0, eNoAction);
								vTaskDelay(pdMS_TO_TICKS(100));
								xTaskNotify(rtc_handle, 0, eSetValueWithOverwrite);
								continue;
							}
						}
						else if(curr_state == state_rtc_set_time_hour_24)
						{
							int hour = 0;
							sscanf(user_msg, "%d", &hour);
							if(hour >= 0 && hour < 24)
							{
								xQueueSend(qtime_set, &hour, portMAX_DELAY);
								xTaskNotify(rtc_handle, 2, eSetValueWithOverwrite);
								curr_state = state_main_menu;
								break;
							}
							else
							{
								xQueueSend(qprint, &t_err_msg_ptr, portMAX_DELAY);
								xTaskNotify(print_handle, 0, eNoAction);
								vTaskDelay(pdMS_TO_TICKS(100));
								xTaskNotify(rtc_handle, 0, eSetValueWithOverwrite);
								continue;
							}
						}
					}
					if(curr_state == state_main_menu)
					{
						break;
					}

				}


			}
		}
	}
}

static void PRINT_handler(void* parameters)
{
	while(1)
	{
		while(xTaskNotifyWait(0, 0, 0, portMAX_DELAY) == pdFAIL);

		if(uxQueueMessagesWaiting(qprint) != 0)
		{
			taskENTER_CRITICAL();
			menu_item* new_item = NULL;
			xQueueReceive(qprint, &new_item, 100);
			taskEXIT_CRITICAL();

			HAL_UART_Transmit(&husart3, (uint8_t*)new_item->msg_item, new_item->len, 100);

			HAL_UART_Receive_IT(&husart3, (uint8_t*)&uart_receive, 1);
		}
	}
}


static void LED_handler(void* parameters)
{
	uint32_t user_cmd = 0;
		while(1)
		{
			while(xTaskNotifyWait(0, 0, 0, portMAX_DELAY) == pdFAIL);

			if(user_cmd == (uint32_t)'#')
			{
				continue;
			}

			menu_item led_menu = {task1_menu, strlen(task1_menu)};
			menu_item* led_menu_ptr = &led_menu;
			taskENTER_CRITICAL();
			xQueueSend(qprint, &led_menu_ptr, portMAX_DELAY);
			xTaskNotify(print_handle, 0, eNoAction);
			taskEXIT_CRITICAL();

			while(xTaskNotifyWait(0, 0, &user_cmd, portMAX_DELAY) == pdFAIL);

			while(1)
			{

				xTaskNotifyWait(0, 0, &user_cmd, 100);

				if(user_cmd == (uint32_t)'#')
				{
					user_cmd = 0;
					break;
				}
				else if(user_cmd == 1)
				{
					HAL_GPIO_WritePin(GPIOB, LD1_Pin, ENABLE);
					HAL_GPIO_WritePin(GPIOB, LD2_Pin, ENABLE);
					HAL_GPIO_WritePin(GPIOB, LD3_Pin, ENABLE);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOB, LD1_Pin, DISABLE);
					HAL_GPIO_WritePin(GPIOB, LD2_Pin, DISABLE);
					HAL_GPIO_WritePin(GPIOB, LD3_Pin, DISABLE);
					HAL_Delay(500);
				}
				else if(user_cmd == 2)
				{
					HAL_GPIO_WritePin(GPIOB, LD1_Pin, ENABLE);
					HAL_Delay(200);
					HAL_GPIO_WritePin(GPIOB, LD2_Pin, ENABLE);
					HAL_Delay(200);
					HAL_GPIO_WritePin(GPIOB, LD3_Pin, ENABLE);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOB, LD1_Pin, DISABLE);
					HAL_Delay(200);
					HAL_GPIO_WritePin(GPIOB, LD2_Pin, DISABLE);
					HAL_Delay(200);
					HAL_GPIO_WritePin(GPIOB, LD3_Pin, DISABLE);
					HAL_Delay(500);
				}
				else if(user_cmd == 3)
				{
					HAL_GPIO_WritePin(GPIOB, LD1_Pin, ENABLE);
					HAL_GPIO_WritePin(GPIOB, LD3_Pin, ENABLE);
					HAL_GPIO_WritePin(GPIOB, LD2_Pin, DISABLE);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOB, LD1_Pin, DISABLE);
					HAL_GPIO_WritePin(GPIOB, LD3_Pin, DISABLE);
					HAL_GPIO_WritePin(GPIOB, LD2_Pin, ENABLE);
					HAL_Delay(500);
				}
			}

		}
}

static void RTC_handler(void* parameters)
{

	t_hrtc_time.Seconds = 00;
	t_hrtc_time.Minutes = 00;
	t_hrtc_time.Hours = 00;
	t_hrtc_time.TimeFormat = RTC_HOURFORMAT12_AM;

	HAL_RTC_SetTime(&hrtc, &t_hrtc_time, RTC_FORMAT_BIN);

	while(1)
	{
		print_time[0] = '\0';

		while(xTaskNotifyWait(0, 0, 0, portMAX_DELAY) != pdPASS);

		get_curr_time();
		strcat(print_time, task2_menu);
		strcat(print_time, current_time);
		menu_item rtc_menu = {print_time, strlen(print_time)};
		menu_item *rtc_menu_ptr = &rtc_menu;
		xQueueSend(qprint, &rtc_menu_ptr, portMAX_DELAY);
		xTaskNotify(print_handle, 0, eNoAction);

		uint32_t rtc_user_cmd = 0;

		while(xTaskNotifyWait(0, 0, &rtc_user_cmd, portMAX_DELAY) != pdPASS);

		switch (rtc_user_cmd) {
			case 1:
			{
				//Get Current Time_Date

				get_curr_time();

				menu_item curr_time_date = {current_time_date, strlen(current_time_date)};
				menu_item *curr_time_date_ptr = &curr_time_date;

				xQueueSend(qprint, &curr_time_date_ptr, portMAX_DELAY);
				xTaskNotify(print_handle, 0, eNoAction);
			}
			break;

			case 2:
			{
				//Set Current Time
				int i = 0;
				while(i < 5)
				{
					curr_state = i;
					uint32_t flag = 0;
					xQueueSend(qprint, &task2_set_time_menu_ptr[i], portMAX_DELAY);
					xTaskNotify(print_handle, 0, eNoAction);
					while(xTaskNotifyWait(0,0,&flag,portMAX_DELAY) == pdFAIL);
					if(flag == 1 || flag == 3)
					{
						i++;
					}
					else if(flag == 0)
					{
						;
					}
					else if(flag == 4)
					{
						i = i+2;
					}
					else
					{
						break;
					}
				}

				uint8_t value[4] = {0};
				for(int i = 0; i<4; i++)
				{
					xQueueReceive(qtime_set, &value[i], portMAX_DELAY);
				}
				t_hrtc_time.Seconds = value[0];
				t_hrtc_time.Minutes = value[1];
				t_hrtc_time.TimeFormat = value[2];
				t_hrtc_time.Hours = value[3];

				HAL_RTC_SetTime(&hrtc, &t_hrtc_time, RTC_FORMAT_BIN);

			}
			break;

			default:
				break;
		}
	}
}


static void format_useroption(char *msg, int len)
{
	for(int i = 0; i < 10; i++)
	{
		xQueueReceive(quser_cmd, &msg[i], 100);

		if(msg[i] == '\n')
		{
			msg[i] = '\0';
			break;
		}
	}

    for(int i = 0; i < len; i++)
    {
        if((int)msg[i] >65 && (int)msg[i] < 91)
        {
            msg[i] = msg[i] + 32;
        }
    }

}

static void get_curr_time(void)
{
	HAL_RTC_GetTime(&hrtc, &t_hrtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &t_hrtc_date, RTC_FORMAT_BIN);

	if(hrtc.Init.HourFormat == RTC_HOURFORMAT_12)
	{
		if(t_hrtc_time.TimeFormat == RTC_HOURFORMAT12_AM)
		{
			sprintf(current_time, "%d:%d:%d AM\n", t_hrtc_time.Hours, t_hrtc_time.Minutes, t_hrtc_time.Seconds);
		}
		else
		{
			sprintf(current_time, "%d:%d:%d PM\n", t_hrtc_time.Hours, t_hrtc_time.Minutes, t_hrtc_time.Seconds);
		}
	}
	else
	{
		sprintf(current_time, "%d:%d:%d\n", t_hrtc_time.Hours, t_hrtc_time.Minutes, t_hrtc_time.Seconds);
	}

	switch (t_hrtc_date.WeekDay) {
		case RTC_WEEKDAY_MONDAY:
					strcat(current_date, "Mon, ");
					break;
		case RTC_WEEKDAY_TUESDAY:
					strcat(current_date, "Tue, ");
					break;
		case RTC_WEEKDAY_WEDNESDAY:
					strcat(current_date, "Wed, ");
					break;
		case RTC_WEEKDAY_THURSDAY:
					strcat(current_date, "Thu, ");
					break;
		case RTC_WEEKDAY_FRIDAY:
					strcat(current_date, "Fri, ");
					break;
		case RTC_WEEKDAY_SATURDAY:
					strcat(current_date, "Sat, ");
					break;
		case RTC_WEEKDAY_SUNDAY:
					strcat(current_date, "Sun, ");
					break;
		default:
			break;
	}

	char date[5] = {0};
	sprintf(date, "%d", t_hrtc_date.Date);
	strcat(current_date, date);

	switch (t_hrtc_date.Month) {
		case RTC_MONTH_JANUARY:
				strcat(current_date, "JAN, ");
				break;
		case RTC_MONTH_FEBRUARY:
				strcat(current_date, "FEB, ");
				break;
		case RTC_MONTH_MARCH:
				strcat(current_date, "MAR, ");
				break;
		case RTC_MONTH_APRIL:
				strcat(current_date, "APR, ");
				break;
		case RTC_MONTH_MAY:
				strcat(current_date, "MAY, ");
				break;
		case RTC_MONTH_JUNE:
				strcat(current_date, "JUN, ");
				break;
		case RTC_MONTH_JULY:
				strcat(current_date, "JUL, ");
				break;
		case RTC_MONTH_AUGUST:
				strcat(current_date, "AUG, ");
				break;
		case RTC_MONTH_SEPTEMBER:
				strcat(current_date, "SEP, ");
				break;
		case RTC_MONTH_OCTOBER:
				strcat(current_date, "OCT, ");
				break;
		case RTC_MONTH_NOVEMBER:
				strcat(current_date, "NOV, ");
				break;
		case RTC_MONTH_DECEMBER:
				strcat(current_date, "DEC, ");
				break;
		default:
			break;
	}

	char year[5] = {0};
	sprintf(year, "%d", t_hrtc_date.Year);
	current_date[0] = '\0';
	strcat(current_date, year);

	sprintf(current_time_date, "%s %s\n", current_time, current_date);

}

void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(&husart3);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		if(uart_receive != '\n')
		{
			xQueueSendFromISR(quser_cmd, &uart_receive, pdFALSE);
		}
		else
		{
			xQueueSendFromISR(quser_cmd, &uart_receive, pdFALSE);
			xTaskNotifyFromISR(cmd_handle, 0, eNoAction, pdFALSE);
		}
		HAL_UART_Receive_IT(&husart3, (uint8_t*)&uart_receive, 1);
	}
}

/* USER CODE END 4 */

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
  if (htim->Instance == TIM6) {
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
