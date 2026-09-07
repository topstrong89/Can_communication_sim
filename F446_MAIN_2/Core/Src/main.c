/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "i2c-lcd.h"
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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define CAN_ID_MAIN_TO_ABS       0x111u
#define CAN_ID_ABS_TO_MAIN       0x121u
#define CAN_ID_AIRBAG_TO_MAIN    0x122u

#define UART_PRINT_TIMEOUT_MS    100U
#define ADC_POLL_TIMEOUT_MS      10U
#define TASK_PERIOD_MS           100U

#define LCD_20X04_COLS          20U
#define LCD_20X04_BUF_LEN       21U

char lcd_line0[LCD_20X04_BUF_LEN];
char lcd_line1[LCD_20X04_BUF_LEN];
char lcd_line2[LCD_20X04_BUF_LEN];
char lcd_line3[LCD_20X04_BUF_LEN];

uint8_t speed_left_cmps = 0U;
uint8_t speed_right_cmps = 0U;
uint8_t speed_avg_cmps = 0U;
uint8_t speed_display = 0U;
uint8_t blink_state = 0U;

#define MOVAVG_OLD_WEIGHT        3U
#define MOVAVG_DIVISOR           4U
#define DISPLAY_ZERO_THRESHOLD   2U

char buff[128];

CAN_FilterTypeDef sFilterConfig;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxDataABS[8] = {0u};

uint8_t main_to_abs_cnt = 0u;

volatile uint32_t adc_val = 0u;
volatile uint8_t sw1 = 0u;     // brake
volatile uint8_t sw2 = 0u;      // direction
volatile uint8_t dir_sw = 0u;   // 0=FWD, 1=REV

volatile uint8_t abs1 = 0u;
volatile uint8_t abs2 = 0u;
volatile uint8_t abs3 = 0u;

volatile uint8_t airbag1 = 0u;
volatile uint8_t airbag2 = 0u;
volatile uint8_t airbag3 = 0u;

volatile uint8_t abs_new_data = 0u;
volatile uint8_t airbag_new_data = 0u;
volatile uint8_t emergency_stop = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

void mPrint(const char *format, ...);
void Send_Command_To_ABS(void);
void Update_Rear_LEDs(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void mPrint(const char * format, ...)
{
	va_list args;
	int32_t len;
	(void)memset((void *)buff, 0, sizeof(buff));

	va_start(args, format);
	len = (int32_t)vsnprintf(buff, sizeof(buff), format, args);
	va_end(args);

	if (len > 0)
	{
		uint16_t tx_len = (len >= (int32_t)sizeof(buff)) ? (uint16_t)(sizeof(buff) - 1U) : (uint16_t)len;
		(void)HAL_UART_Transmit(&huart2, (uint8_t *)buff, tx_len, UART_PRINT_TIMEOUT_MS);
	}
}

void Read_Local_Inputs(void)
{
	//Read pot
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, ADC_POLL_TIMEOUT_MS);
	adc_val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	//PC2 brake
	sw1 = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) ? 1U : 0U;

	//PC3 direction
	sw2 = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET) ? 1U : 0U;
	dir_sw = sw2;
}

void Send_Command_To_ABS(void)
{
	uint32_t adc_to_send;
	uint8_t brake_to_send;
	uint8_t dir_to_send;
	HAL_StatusTypeDef can_status;

	TxHeader.StdId = CAN_ID_MAIN_TO_ABS;
	TxHeader.ExtId = 0u;
	TxHeader.IDE   = CAN_ID_STD;
	TxHeader.RTR   = CAN_RTR_DATA;
	TxHeader.DLC   = 4u;
	TxHeader.TransmitGlobalTime = DISABLE;

	Read_Local_Inputs();

	adc_to_send   = adc_val;
	brake_to_send = sw1;
	dir_to_send   = sw2;

	if (emergency_stop)
	{
		adc_to_send   = 0u;
		brake_to_send = 1u;
	}

	TxDataABS[0] = (uint8_t)(adc_to_send & 0xFFu);         // low byte
	TxDataABS[1] = (uint8_t)((adc_to_send >> 8) & 0x0Fu);  // high byte
	TxDataABS[2] = brake_to_send;                         // brake
	TxDataABS[3] = dir_to_send;                           // dir

	can_status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxDataABS, &TxMailbox);

	if (can_status == HAL_OK)
	{
		if (emergency_stop != 0U)
		{
			mPrint("[MAIN->ABS] EMERGENCY STOP | RAW=%02X %02X %02X %02X\r\n",
					TxDataABS[0],
					TxDataABS[1],
					TxDataABS[2],
					TxDataABS[3]);
		}
		else
		{
			mPrint("[MAIN TX ABS] ID=0x111 | ADC=%lu | BRAKE=%u | DIR=%u | RAW=%02X %02X %02X %02X\r\n",
					adc_to_send,
					brake_to_send,
					dir_to_send,
					TxDataABS[0],
					TxDataABS[1],
					TxDataABS[2],
					TxDataABS[3]);
		}

		main_to_abs_cnt++;
	}
	else
	{
		mPrint("[MAIN TX ABS] FAIL | err=0x%08lX | free=%lu\r\n",
				HAL_CAN_GetError(&hcan1),
				HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
	HAL_StatusTypeDef rx_status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	if (rx_status == HAL_OK && RxHeader.IDE == CAN_ID_STD) //STD-11bits
	{
		if (RxHeader.StdId == CAN_ID_ABS_TO_MAIN) //0x121
		{
			if (RxHeader.DLC >= 3U)
			{
				abs1 = RxData[0];
				abs2 = RxData[1];
				abs3 = RxData[2];
				abs_new_data = 1U;
			}
		}
		else if (RxHeader.StdId == CAN_ID_AIRBAG_TO_MAIN) //0x122
		{
			if (RxHeader.DLC >= 1U)
			{
				airbag1 = RxData[0];
				if (airbag1 == 1u) {
					emergency_stop = 1u;
				} else {
					emergency_stop = 0u;
				}
				airbag_new_data = 1U;
			}
		}
	}
}

void Update_Rear_LEDs(void)
{
	if (sw1 == 1U)//brake
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
	}
	else if (dir_sw == 1U)//backward
	{
		//500ms on-off
		if ((HAL_GetTick() / 500U) % 2U == 0U)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
		}
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
	}
}

void Update_LCD(void)
{
	static uint32_t last_lcd_update = 0;

	if (HAL_GetTick() - last_lcd_update >= 150)
	{
		last_lcd_update = HAL_GetTick();

		char buf0[21] = {0};
		char buf1[21] = {0};
		char buf2[21] = {0};
		char buf3[21] = {0};

		if (emergency_stop == 1u)
		{
			sprintf(buf0, "!!! EMERGENCY !!!");
			sprintf(buf1, "  AIRBAG DEPLOYED!  ");
			sprintf(buf2, "  SYSTEM LOCKED  ");
			sprintf(buf3, "  ACTION NEEDED  ");
		}
		else
		{
			char* dir_str = (dir_sw == 0) ? "FWD" : "REV";
			sprintf(buf0, "	NODE MAIN	");


			uint8_t throttle_pct = (adc_val * 100) / 4095;
			uint8_t num_blocks = (adc_val * 10) / 4095;
			if (num_blocks > 10) num_blocks = 10;

			char bar[11];
			for(int i = 0; i < 10; i++) {
				if (i < num_blocks) {
					bar[i] = (char)0xFF;
				} else {
					bar[i] = '-';
				}
			}
			bar[10] = '\0';
			sprintf(buf1, "THR:[%s] %3d%%", bar, throttle_pct);

			sprintf(buf2, "SPD: %3d  DIR:%s", abs1, dir_str);

			char* brk_str = (sw1 == 1) ? "ON " : "OFF";
			sprintf(buf3, "BRK:%s  TX_CNT: %3d", brk_str, main_to_abs_cnt);
		}

		char* buffers[] = {buf0, buf1, buf2, buf3};
		for (int j = 0; j < 4; j++) {
			int len = strlen(buffers[j]);
			for (int i = len; i < 20; i++) {
				buffers[j][i] = ' '; // Điền khoảng trắng
			}
			buffers[j][20] = '\0';   // Đảm bảo kết thúc chuỗi đúng chuẩn
		}

		lcd_put_cur(0, 0); lcd_send_string(buf0);
		lcd_put_cur(1, 0); lcd_send_string(buf1);
		lcd_put_cur(2, 0); lcd_send_string(buf2);
		lcd_put_cur(3, 0); lcd_send_string(buf3);
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
	MX_CAN1_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	mPrint("\r\n=============================\r\n");
	mPrint("BOOT OK F446RE MAIN NODE\r\n");

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //32bit
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	sFilterConfig.FilterIdHigh      = (CAN_ID_ABS_TO_MAIN << 5);//0x121
	sFilterConfig.FilterIdLow       = (CAN_ID_AIRBAG_TO_MAIN << 5);//0x122
	sFilterConfig.FilterMaskIdHigh  = 0x0000;
	sFilterConfig.FilterMaskIdLow   = 0x0000;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		mPrint("CAN Filter Config FAIL\r\n");
		Error_Handler();
	}
	mPrint("CAN Filter Config OK\r\n");

	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		mPrint("CAN Start FAIL\r\n");
		Error_Handler();
	}
	mPrint("CAN Start OK\r\n");

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		mPrint("CAN Notification FAIL\r\n");
		Error_Handler();
	}
	mPrint("CAN Notification OK\r\n");

	//NVIC
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	mPrint("CAN RX0 NVIC OK\r\n");

	mPrint("MAIN READY | TX:0x111 | RX:0x121 | RX:0x122\r\n");
	mPrint("=============================\r\n\r\n");

	lcd_init();
	lcd_put_cur(0, 0);
	lcd_send_string("MAIN NODE READY ");
	HAL_Delay(1000);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		Send_Command_To_ABS();

		Update_Rear_LEDs();

		if (abs_new_data != 0U)
		{
			abs_new_data = 0U;
			mPrint("[MAIN RX ABS] ID=0x121 | D0=%u | D1=%02X | D2=%02X\r\n", abs1, abs2, abs3);
		}

		if (airbag_new_data != 0U)
		{
			airbag_new_data = 0U;
			if (emergency_stop == 1u) {
				mPrint("\r\n[!!!] AIRBAG DEPLOYED! EMERGENCY [!!!]\r\n");
			} else {
				mPrint("[MAIN RX AIRBAG] ID=0x122 | Status: SAFE\r\n\n");
			}
		}

		Update_LCD();

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
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */
	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// PB8 = CAN_RX, PB9 = CAN_TX
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 7;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Cấu hình PB10 cho I2C2_SCL
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Cấu hình PC12 cho I2C2_SDA
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

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
	HAL_GPIO_WritePin(GPIOC, LED0_Pin|LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED0_Pin LED1_Pin */
	GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Brake_Pin Direction_Pin */
	GPIO_InitStruct.Pin = Brake_Pin|Direction_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
void CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);
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
