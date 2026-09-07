/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : AIRBAG Node - TX only
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
#include <stdbool.h>
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
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define CS_ENABLE()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define CS_DISABLE() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
extern SPI_HandleTypeDef hspi1;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Acceleration_t;

Acceleration_t accel_data;

float x_mg = 0.0f;
float y_mg = 0.0f;
float z_mg = 0.0f;
float total_force = 0.0f;

volatile uint8_t data_ready_flag = 0;
volatile uint8_t deploy_flag = 0;

#define CAN_ID_AIRBAG_TO_MAIN    0x122U
#define LIS3DSH_CTRL_REG4        0x20u
#define LIS3DSH_CTRL_REG4_CFG    0x67u
#define LIS3DSH_CTRL_REG5        0x23u
#define LIS3DSH_CTRL_REG5_CFG    0xC8u

#define LIS3DSH_OUT_X_L          0x28u
#define LIS3DSH_OUT_X_H          0x29u
#define LIS3DSH_OUT_Y_L          0x2Au
#define LIS3DSH_OUT_Y_H          0x2Bu
#define LIS3DSH_OUT_Z_L          0x2Cu
#define LIS3DSH_OUT_Z_H          0x2Du

#define ACCEL_SCALE_FACTOR       0.06f
#define FORCE_THRESHOLD          6250000.0f //2500mg ^ 2

#define UART_PRINT_TIMEOUT_MS    100U

static char buff[128];

CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxData[8] = {0u};

//Airbag data
uint8_t airbag1 = 0u;
uint8_t airbag2 = 0u;
uint8_t airbag3 = 0u;

uint8_t airbag_to_main_cnt = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void mPrint(const char *format, ...);
void Send_Data_To_MAIN(void);
void Print_AIRBAG_TX(void);

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  extern UART_HandleTypeDef huart2;
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
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

void Print_AIRBAG_TX(void)
{
    mPrint("[AIRBAG TX] ID=0x122 | AB1=%u | AB2=%u | AB3=%u | CNT=%u | RAW=%02X %02X %02X\r\n",
           TxData[0], TxData[1], TxData[2], airbag_to_main_cnt,
           TxData[0], TxData[1], TxData[2]);
}

void Send_Data_To_MAIN(void)
{
	HAL_StatusTypeDef tx_status;

    TxHeader.StdId = CAN_ID_AIRBAG_TO_MAIN;   // 0x122
    TxHeader.ExtId = 0u;
    TxHeader.IDE   = CAN_ID_STD;
    TxHeader.RTR   = CAN_RTR_DATA;
    TxHeader.DLC   = 1u;
    TxHeader.TransmitGlobalTime = DISABLE;

    TxData[0] = deploy_flag;   //

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0u)
    {
        tx_status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

        if (tx_status == HAL_OK)
        {
            airbag_to_main_cnt++;
            Print_AIRBAG_TX();
        }
        else
        {
            mPrint("[AIRBAG TX] FAIL | err=0x%08lX | free=%lu\r\n",
                   HAL_CAN_GetError(&hcan1),
                   HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
        }
    }
    else
    {
        mPrint("[ERROR] CAN is unsync\r\n");
        (void)HAL_CAN_AbortTxRequest(&hcan1,
                                     CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
}

static void MEMS_WriteReg(uint8_t regAddr, uint8_t data) {
    uint8_t spiData[2];
    spiData[0] = regAddr & 0x7Fu; //write
    spiData[1] = data;

    CS_ENABLE();
    HAL_SPI_Transmit(&hspi1, spiData, 2u, HAL_MAX_DELAY);//send 2 bytes
    CS_DISABLE();
}

uint8_t MEMS_ReadReg(uint8_t regAddr) {
    uint8_t txData = regAddr | 0x80u; //read
    uint8_t rxData = 0u;

    CS_ENABLE();
    HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rxData, 1, HAL_MAX_DELAY);
    CS_DISABLE();

    return rxData;
}

static void LIS3DSH_Init(void) {

    MEMS_WriteReg(LIS3DSH_CTRL_REG4, LIS3DSH_CTRL_REG4_CFG);//0110 0111

    MEMS_WriteReg(LIS3DSH_CTRL_REG5, LIS3DSH_CTRL_REG5_CFG);//1100 1000
}

static void LIS3DSH_ReadAccel(Acceleration_t *accel) {
    uint8_t x_l, x_h, y_l, y_h, z_l, z_h;

    x_l = MEMS_ReadReg(LIS3DSH_OUT_X_L); //OUT_X_L
    x_h = MEMS_ReadReg(LIS3DSH_OUT_X_H); //OUT_X_H

    y_l = MEMS_ReadReg(LIS3DSH_OUT_Y_L); //OUT_Y_L
    y_h = MEMS_ReadReg(LIS3DSH_OUT_Y_H); //OUT_Y_H

    z_l = MEMS_ReadReg(LIS3DSH_OUT_Z_L); //OUT_Z_L
    z_h = MEMS_ReadReg(LIS3DSH_OUT_Z_H); //OUT_Z_H

    //shift 8 bits of the high reg to left then put the low reg in
    accel->x = (int16_t)((x_h << 8) | x_l);
    accel->y = (int16_t)((y_h << 8) | y_l);
    accel->z = (int16_t)((z_h << 8) | z_l);
}

static void CAN_Filter_Config(void) {
    CAN_FilterTypeDef canfilterconfig;
    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 0u;
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfilterconfig.FilterIdHigh = 0u;
    canfilterconfig.FilterIdLow = 0u;
    canfilterconfig.FilterMaskIdHigh = 0u;
    canfilterconfig.FilterMaskIdLow = 0u;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  LIS3DSH_Init();

  CAN_Filter_Config();
  HAL_CAN_Start(&hcan1);


  TxHeader.StdId = 0x110;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 1;
  TxHeader.TransmitGlobalTime = DISABLE;

  mPrint("\r\nAIRBAG NODE INIT OK\r\n");
  mPrint("\r\n=============================\r\n");
  mPrint("BOOT OK\r\n");
  mPrint("UART2 OK\r\n");

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      mPrint("CAN Start FAIL\r\n");
  }
  else
  {
      mPrint("CAN Start OK\r\n");
  }

  mPrint("AIRBAG NODE READY | TX:0x122\r\n");
  mPrint("=============================\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t last_main_loop_time = 0U;

  static uint32_t btn_debounce_time = 0U;
  static uint8_t btn_fsm_state = 0U; // 0: IDLE, 1: DEBOUNCE, 2: WAIT_RELEASE

  while (1)
  {

	  uint32_t current_time = HAL_GetTick();

	  GPIO_PinState btn_pin_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	  switch (btn_fsm_state)
	  {
	  case 0U: //IDLE
		  if (btn_pin_status == GPIO_PIN_SET)
		  {
			  btn_debounce_time = current_time; //update time
			  btn_fsm_state = 1U;               //sang debounce de kiem tra tg de nut
		  }
		  break;

	  case 1U: //debounce >50ms
		  if (btn_pin_status == GPIO_PIN_SET)
		  {
			  if ((current_time - btn_debounce_time) >= 50U)
			  {
				  deploy_flag = 0U;
//				  if (deploy_flag == 0U)
//				  {
//					  deploy_flag = 1U;
//					  mPrint("[AIRBAG] Kich hoat khan cap bang nut nhan (Test Mode).\r\n");
//				  }
//				  else
//				  {
//					  deploy_flag = 0U;
//					  mPrint("[AIRBAG] He thong da duoc reset ve trang thai an toan.\r\n");
//				  }
				  btn_fsm_state = 2U;
			  }
		  }
		  else
		  {
			  btn_fsm_state = 0U; //truong hop nhieu
		  }
		  break;

	  case 2U: //WAIT_RELEASE
		  if (btn_pin_status == GPIO_PIN_RESET)
		  {
			  btn_fsm_state = 0U; //back to IDLE
		  }
		  break;

	  default:
		  btn_fsm_state = 0U; //default IDLE
		  break;
	  }

	  //airbag_deploymment
	  if ((current_time - last_main_loop_time) >= 10U)
	        {
	            //update time period
	            last_main_loop_time = current_time;

	            if (deploy_flag == 0U)
	            {
	                LIS3DSH_ReadAccel(&accel_data);

	                x_mg = accel_data.x * ACCEL_SCALE_FACTOR; //factor = 0.06
	                y_mg = accel_data.y * ACCEL_SCALE_FACTOR;
	                z_mg = accel_data.z * ACCEL_SCALE_FACTOR;
	                total_force = (x_mg * x_mg) + (y_mg * y_mg) + (z_mg * z_mg);

	                if (total_force > FORCE_THRESHOLD)//6250000 = 2500mg ^ 2
	                {
	                    deploy_flag = 1U;
	                    TxData[0] = deploy_flag;
	                    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	                    mPrint("[COLLISION DETECTED]");
	                }
	            }
	            //deployed
	            if (deploy_flag == 1U)
	            {
	                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	            }
	            else
	            {
	                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	            }

	            Send_Data_To_MAIN();
	        }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
