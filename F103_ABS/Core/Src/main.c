/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : ABS Node - RX from MAIN and TX back to MAIN
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ID_MAIN_TO_ABS           0x111U
#define CAN_ID_ABS_TO_MAIN           0x121U

#define UART_PRINT_TIMEOUT_MS        100U

#define WHEEL_DIAMETER_MM            65.0f
#define WHEEL_CIRCUM_MM              3.1415926f * WHEEL_DIAMETER_MM

#define ENCODER_PPR_MOTOR            11.0f
#define GEAR_RATIO                   21.3f


#define ENCODER_MULTIPLIER           4.0f

#define COUNTS_PER_WHEEL_REV         ENCODER_PPR_MOTOR * GEAR_RATIO * ENCODER_MULTIPLIER

#define SPEED_SAMPLE_TIME_MS         100U
#define SPEED_SAMPLE_TIME_S          0.1f

#define CAN_TX_PERIOD_MS             100U

#define PWM_DEAD_ADC                 10U
#define PWM_MIN_PERCENT              50U
#define PWM_MAX_PERCENT              80U

#define ADC_FULL_SCALE               4095U

#define COUNTER_MAX_16BIT            65535L
#define COUNTER_HALF_16BIT           32767L
#define COUNTER_HALF_NEG_16BIT       -32768L
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static CAN_FilterTypeDef sFilterConfig;
static CAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];

static CAN_TxHeaderTypeDef TxHeader;
static uint32_t TxMailbox;
static uint8_t TxData[8] = {0U};

static char buff[128];

// data Rx MAIN
static uint8_t main_cmd1 = 0U;
static uint8_t main_cmd2 = 0U;
static uint8_t main_sw1  = 0U;
static uint8_t main_sw2  = 0U;

// data ABS Tx MAIN
static uint8_t abs1 = 0U;
static uint8_t abs2 = 0U;
static uint8_t abs3 = 0U;

// Rx flag
static volatile uint8_t abs_rx_new_data = 0U;

// counter
static uint8_t abs_to_main_cnt = 0U;

// motor
static uint16_t pwm_motor1 = 0U;
static uint8_t motor_state = 0U;// 0 stop, 1 forward, 2 reverse, 3 brake

// encoder
static int32_t enc1_now = 0L;
static int32_t enc1_prev = 0L;
static int32_t enc1_diff = 0L;

static uint8_t wheel1_speed = 0U;

static float wheel1_speed_mps = 0.0f;

static uint32_t speed_last_tick = 0U;
static uint32_t can_tx_last_tick = 0U;

static uint16_t adc_raw_from_main = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void mPrint(const char * format, ...);
//static void Print_MAIN_RX(void);
static void Send_Data_To_MAIN(void);
static void Print_ABS_TX(void);

static uint16_t Map_ADC_To_PWM(uint16_t adc_raw);
static void Motor_Stop(void);
static void Motor_Forward(uint16_t pwm);
static void Motor_Reverse(uint16_t pwm);
static void Motor_Control_From_MAIN(void);
static void Encoder_Update(void);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void mPrint(const char * format, ...)
{
	va_list args;
	int32_t len;

	(void)memset((void *)buff, 0, sizeof(buff));

	va_start(args, format);
	len = (int32_t)vsnprintf(buff, sizeof(buff), format, args);
	va_end(args);

	if (len > 0)
	{
		uint16_t tx_len;

		if (len >= (int32_t)sizeof(buff))
		{
			tx_len = (uint16_t)(sizeof(buff) - 1U);
		}
		else
		{
			tx_len = (uint16_t)len;
		}

		(void)HAL_UART_Transmit(&huart2, (uint8_t *)buff, tx_len, UART_PRINT_TIMEOUT_MS);
	}
	else
	{
	}
}

//static void Print_MAIN_RX(void)
//{
//	mPrint("[ABS RX MAIN] ID=0x111 | ADC=%u | BRAKE=%u | DIR=%u\r\n",
//			adc_raw_from_main, main_sw1, main_sw2);
//}

static void Print_ABS_TX(void)
{
	mPrint("[ABS RX MAIN] ID=0x111 | ADC=%u | BRAKE=%u | DIR=%u\r\n",
				adc_raw_from_main, main_sw1, main_sw2);
	mPrint("[ABS TX MAIN] ID=0x121 | D1=%u cm/s | D2=%u cm/s | STATE=%u | CNT=%u\r\n",
			TxData[0], TxData[1], TxData[2], abs_to_main_cnt);
	mPrint("[ABS MOTOR] ADC=%u | PWM=%u | BRAKE=%u | DIR=%u\r\n\n",
				adc_raw_from_main, pwm_motor1, main_sw1, main_sw2);
}

static uint16_t Map_ADC_To_PWM(uint16_t adc_raw)
{
	uint32_t arr;
	uint32_t pwm_min;
	uint32_t pwm_max;
	uint32_t ccr;
	uint16_t pwm_out;

	arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
	pwm_min = (arr * PWM_MIN_PERCENT) / 100U;
	pwm_max = (arr * PWM_MAX_PERCENT) / 100U;

	if (adc_raw <= PWM_DEAD_ADC)
	{
		pwm_out = 0U;
	}
	else
	{
		ccr = pwm_min +
				(((uint32_t)(adc_raw - PWM_DEAD_ADC)) * (pwm_max - pwm_min)) /
				(ADC_FULL_SCALE - PWM_DEAD_ADC);

		if (ccr > pwm_max)
		{
			ccr = pwm_max;
		}
		else{}
		pwm_out = (uint16_t)ccr;
	}

	return pwm_out;
}

static void Motor_Forward(uint16_t pwm)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)pwm);
	motor_state = 1U;
}

static void Motor_Reverse(uint16_t pwm)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)pwm);
	motor_state = 2U;
}

static void Motor_Stop(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
	motor_state = 0U;
}

static void Motor_Control_From_MAIN(void)
{
	uint16_t pwm;

	pwm = Map_ADC_To_PWM(adc_raw_from_main);

	pwm_motor1 = pwm;

//	mPrint("[ABS MOTOR] ADC=%u | PWM=%u | BRAKE=%u | DIR=%u\r\n\n",
//			adc_raw_from_main, pwm, main_sw1, main_sw2);

	if (main_sw1 == 1U)
	{
		Motor_Stop();
		motor_state = 3U;
	}
	else if (adc_raw_from_main == 0U)
	{
		Motor_Stop();
	}
	else if (main_sw2 == 0U)
	{
		Motor_Forward(pwm);
	}
	else
	{
		Motor_Reverse(pwm);
	}
}

static void Encoder_Update(void)
{
	uint32_t now_tick = HAL_GetTick();

	if ((now_tick - speed_last_tick) >= SPEED_SAMPLE_TIME_MS)
	{
		speed_last_tick = now_tick;

		enc1_now  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
		enc1_diff = enc1_now - enc1_prev;

		//overflow correction
		if (enc1_diff >  COUNTER_HALF_16BIT) enc1_diff -= (COUNTER_MAX_16BIT + 1L);
		if (enc1_diff < COUNTER_HALF_NEG_16BIT) enc1_diff += (COUNTER_MAX_16BIT + 1L);

		enc1_prev = enc1_now;

		if (enc1_diff < 0L) enc1_diff = -enc1_diff;

		float rev1 = (float)enc1_diff / COUNTS_PER_WHEEL_REV;

		float speed1_cmps = (rev1 * WHEEL_CIRCUM_MM / 1000.0f / SPEED_SAMPLE_TIME_S) * 100.0f;

		wheel1_speed = (uint8_t)((speed1_cmps > 255.0f) ? 255.0f : speed1_cmps);
		abs1 = wheel1_speed;
		abs2 = 0x00;
		abs3 = motor_state;
	}
}

static void Send_Data_To_MAIN(void)
{
	HAL_StatusTypeDef tx_status;
	TxHeader.StdId = CAN_ID_ABS_TO_MAIN; // 0x121
	TxHeader.ExtId = 0U;
	TxHeader.IDE   = CAN_ID_STD;
	TxHeader.RTR   = CAN_RTR_DATA;
	TxHeader.DLC   = 3U;
	TxHeader.TransmitGlobalTime = DISABLE;

	TxData[0] = abs1;
	TxData[1] = abs2;
	TxData[2] = abs3;

	tx_status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

	if (tx_status == HAL_OK) {
		abs_to_main_cnt++;
		Print_ABS_TX();
	} else {
		mPrint("[ABS TX MAIN] FAIL | err=0x%08lX\r\n", HAL_CAN_GetError(&hcan));
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan_ptr)
{
	HAL_StatusTypeDef rx_status;

	rx_status = HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &RxHeader, RxData);

	if (rx_status == HAL_OK)
	{
		if (RxHeader.IDE == CAN_ID_STD)
		{
			switch (RxHeader.StdId)
			{
			case CAN_ID_MAIN_TO_ABS:
			{
				if (RxHeader.DLC >= 4U)
				{
					main_cmd1 = RxData[0];
					main_cmd2 = RxData[1];
					main_sw1 = RxData[2];
					main_sw2 = RxData[3];

					adc_raw_from_main =
							(uint16_t)((uint16_t)main_cmd1 |
									(uint16_t)(((uint16_t)main_cmd2 & 0x0FU) << 8U));

					abs_rx_new_data = 1U;
				}
				else{}
				break;
			}

			default:
			{
				uint8_t i;

				mPrint("[ABS RX] ID=0x%03lX DLC=%u | ",
						RxHeader.StdId, RxHeader.DLC);

				for (i = 0U; i < RxHeader.DLC; i++)
				{
					mPrint("%02X ", RxData[i]);
				}

				mPrint("\r\n");
				break;
			}
			}
		}
		else{}
	}
	else{}
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
	MX_CAN_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	mPrint("\r\n=============================\r\n");
	mPrint("BOOT OK\r\n");
	mPrint("UART2 OK\r\n");

	sFilterConfig.FilterBank = 0U;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14U;

	sFilterConfig.FilterIdHigh = (CAN_ID_MAIN_TO_ABS << 5); //0x111
	sFilterConfig.FilterIdLow = 0x0000U;
	sFilterConfig.FilterMaskIdHigh = 0xFFE0U;
	sFilterConfig.FilterMaskIdLow = 0x0000U;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		mPrint("CAN Filter Config FAIL\r\n");
		Error_Handler();
	}
	else
	{
		mPrint("CAN Filter Config OK\r\n");
	}

	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
		mPrint("CAN Start FAIL\r\n");
		Error_Handler();
	}
	else
	{
		mPrint("CAN Start OK\r\n");
	}

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		mPrint("CAN Notification FAIL\r\n");
		Error_Handler();
	}
	else
	{
		mPrint("CAN Notification OK\r\n");
	}

	HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
	mPrint("CAN RX1 NVIC OK\r\n");

	mPrint("ABS NODE READY| RX:0x111 | TX:0x121\r\n");
	mPrint("=============================\r\n\r\n");

	(void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	(void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	(void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	Motor_Stop();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (abs_rx_new_data != 0U)
		{
			abs_rx_new_data = 0U;
//			Print_MAIN_RX();
			Motor_Control_From_MAIN();
		}
		else{}

		Encoder_Update();

		if ((HAL_GetTick() - can_tx_last_tick) >= CAN_TX_PERIOD_MS)
		{
			can_tx_last_tick = HAL_GetTick();
			Send_Data_To_MAIN();
		}
		else{}

		HAL_Delay(5U);
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
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 6;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 35995;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA4 PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
