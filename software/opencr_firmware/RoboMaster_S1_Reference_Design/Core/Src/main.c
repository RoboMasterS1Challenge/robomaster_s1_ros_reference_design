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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "robomaster_s1.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "command.pb.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_RX_DATA_SIZE  2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

uint8_t timer10sec_flag;
uint32_t timer10sec_counter;
uint8_t timer1sec_flag;
uint32_t timer1sec_counter;
uint8_t timer100msec_flag;
uint32_t timer100msec_counter;
uint8_t timer10msec_flag;
uint32_t timer10msec_counter;
uint8_t timer1msec_flag;
uint32_t timer1msec_counter;

CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader2;
uint8_t TxData2[8];
uint8_t RxData2[8];
uint32_t TxMailbox2;
uint32_t RxMailbox2;

volatile CANRxMsg rx_msg_buffer2[BUFFER_SIZE];
volatile int buffer_rp2;
volatile int buffer_wp2;

const uint8_t can_command_0x201[26][81] = {
#include "command_list.csv"
};

extern uint8_t*  usb_rBuf;
extern int  usb_rBuf_wp;
extern int  usb_rBuf_rp;

double base_odom_yaw;

double gimbal_base_yaw_angle;
double gimbal_map_yaw_angle;
double gimbal_base_pitch_angle;
double gimbal_map_pitch_angle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_FilterConfig(void);
extern int parseCanData(uint8_t id, uint8_t *in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t *out_data_size);
extern twist parseTwistCommandData(uint8_t* in_data, uint8_t in_data_size);
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
  MX_TIM2_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  /* Configure the CAN peripheral */
  CAN_FilterConfig();

  /*## Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /*## Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_FULL) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  usb_rBuf_wp = 0;
  usb_rBuf_rp = 0;
  buffer_rp2 = 0;
  buffer_wp2 = 0;
  timer10sec_flag = 0;
  timer10sec_counter = 0;
  timer1sec_flag = 0;
  timer1sec_counter = 0;
  timer100msec_flag = 0;
  timer100msec_counter = 0;
  timer10msec_flag = 0;
  timer10msec_counter = 0;
  timer1msec_flag = 0;
  timer1msec_counter = 0;


  uint8_t buffer[128];
  size_t message_length;
  bool status;

  Command message = Command_init_zero;

  /* Create a stream that will write to our buffer. */
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  /* Fill in the lucky number */
  message.type = 1;
  message.id = 0x55;

  /* Now we are ready to encode the message! */
  status = pb_encode(&stream, Command_fields, &message);
  message_length = stream.bytes_written;

  /* Then just check for any errors.. */
  if (!status)
  {
	  printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));

  }

  int led_flag =0;
  while (1)
  {
	    // 1msec TASK
	    if (timer1msec_flag)
	    {
	      timer1msec_flag = 0;
	      // Send Command
	      // Search Cyclic Command
//	      for(int i = 0; i< size)
	    }

	    // 10msec TASK
	    if (timer10msec_flag)
	    {
	      timer10msec_flag = 0;
	    }

	    // 100msec TASK
	    if (timer100msec_flag)
	    {
	      timer100msec_flag = 0;
	    }

	    // 1sec TASK
	    if (timer1sec_flag)
	    {
	      timer1sec_flag = 0;
	      if(led_flag){
		      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET); // High
		      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET); // High
		      led_flag = 0;
	      }else{
		      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET); // Low
		      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); // Low
		      led_flag = 1;
	      }
	    }

	    // 10sec TASK
	    if (timer10sec_flag)
	    {
	      timer10sec_flag = 0;
	    }

	    uint8_t tmp_usb_rBuf[APP_RX_DATA_SIZE];
	    int idx = 0;
	    while (usb_rBuf_rp != usb_rBuf_wp)
	    {
	    	tmp_usb_rBuf[idx] = usb_rBuf[usb_rBuf_rp];
	    	usb_rBuf_rp++;
	    	idx++;
	    }
		twist command_twist = parseTwistCommandData(tmp_usb_rBuf, TWIST_COMMAND_SIZE);

		// Receive CAN Data
	    if (buffer_rp2 != buffer_wp2)
	    {
	      uint8_t receive_data[BUFFER_SIZE];
	      uint8_t receive_data_size = 0;
	      CANRxMsg msg = rx_msg_buffer2[buffer_rp2];
	      int ret = parseCanData(msg.can_id, msg.data, msg.dlc, receive_data, &receive_data_size);
	      buffer_rp2++;
	      buffer_rp2 %= BUFFER_SIZE;
	      if (ret)
	      {
	    	    switch(msg.can_id){
	    	      case ID_0x201:
	    			break;
	    	      case ID_0x202:
	    	          // From Motion Controller
	    	          if(receive_data[1] == 0x3D &&
	    	        	 receive_data[4] == 0x03 &&
						 receive_data[5] == 0x09)
	    	          {
	    	            int flag = (receive_data[24] >> 7) & 0x01;
	    	            base_odom_yaw = ((((uint16_t)receive_data[24]) << 8) | (((uint16_t)receive_data[23]) << 0));
	    	            if (flag == 0)
	    	            {
	    	              int shift = (0x86 - ((receive_data[24] << 1) | (receive_data[23] >> 7)));
	    	              base_odom_yaw = ((1 << 7) | receive_data[23]) >> shift;
	    	              base_odom_yaw *= -1;
	    	            }
	    	            else
	    	            {
	    	              int shift = (0x186 - ((receive_data[24] << 1) | (receive_data[23] >> 7)));
	    	              base_odom_yaw = ((1 << 7) | receive_data[23]) >> shift;
	    	            }
	    	            if (receive_data[24] == 0 && receive_data[23] == 0)
	    	            {
	    	              base_odom_yaw = 0;
	    	            }
	    	          }
	    	          break;
	    	        case ID_0x203:
	    	          // From Gimbal
	    	          if (receive_data[1] == 0x11 &&
	    	        	  receive_data[4] == 0x04 &&
						  receive_data[5] == 0x03)
	    	          {
	    	            // Yaw Angle
	    	            uint16_t data = receive_data[12];
	    	            data = (data << 8) | receive_data[11];
	    	            gimbal_base_yaw_angle = -(int16_t)(data) / 10.0;
	    	            data = receive_data[14];
	    	            data = (data << 8) | receive_data[13];
	    	            gimbal_map_yaw_angle = -(int16_t)(data) / 100.0;
	    	          }
	    	          if (receive_data[1] == 0x16 &&
	    	        	  receive_data[4] == 0x04 &&
						  receive_data[5] == 0x09)
	    	          {
	    	            // Pitch Angle
	    	            uint32_t data = receive_data[14];
	    	            data = (data << 8) | receive_data[13];
	    	            data = (data << 8) | receive_data[12];
	    	            data = (data << 8) | receive_data[11];
	    	            gimbal_map_pitch_angle = (int32_t)(data) / 20000000.0 * 30.0;
	    	            data = receive_data[18];
	    	            data = (data << 8) | receive_data[17];
	    	            data = (data << 8) | receive_data[16];
	    	            data = (data << 8) | receive_data[15];
	    	            gimbal_base_pitch_angle = (int32_t)(data) / 20000000.0 * 30.0;
	    	          }
	    	          break;
	    			break;
	    	      case ID_0x204:
	    	        //if(out_data[4] ==0x09 && (out_data[5] ==0x17 || out_data[5] ==0x18))
	    	        // {
	    	        // for(uint8_t i = 0; i < out_data_size; i++){
	    	        //   printf("0x%02X,",out_data[i]);
	    	        // }
	    	        // printf("\n");
	    	        // }
	    	      break;
	    	      case ID_0x211:
	    	      case ID_0x212:
	    	      case ID_0x213:
	    	      case ID_0x214:
	    	      case ID_0x215:
	    	      case ID_0x216:
	    	      //   {
	    	      //   for(uint8_t i = 0; i < out_data_size; i++){
	    	      //     printf("0x%02X,",out_data[i]);
	    	      //   }
	    	      //   printf("\n");
	    	      //   }
	    	      break;
	    	      default:
	    	      break;
	    	    }

	      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SYS_USER_LED3_Pin|SYS_USER_LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SYS_STS_LED_Pin|SYS_USER_LED4_Pin|SYS_USER_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SYS_USER_LED3_Pin */
  GPIO_InitStruct.Pin = SYS_USER_LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYS_USER_LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SYS_USER_LED2_Pin */
  GPIO_InitStruct.Pin = SYS_USER_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYS_USER_LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_USER2_Pin */
  GPIO_InitStruct.Pin = SW_USER2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_USER2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_INT_Pin */
  GPIO_InitStruct.Pin = SPI1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_SIG_Pin */
  GPIO_InitStruct.Pin = BUZZER_SIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_SIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUT_USER2_Pin SW_USER1_Pin */
  GPIO_InitStruct.Pin = BUT_USER2_Pin|SW_USER1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : BUT_USER1_Pin */
  GPIO_InitStruct.Pin = BUT_USER1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUT_USER1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_VBUS_Pin OTG_FS_OC_Pin OTG_FS_PWR_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin|OTG_FS_OC_Pin|OTG_FS_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SYS_STS_LED_Pin SYS_USER_LED4_Pin */
  GPIO_InitStruct.Pin = SYS_STS_LED_Pin|SYS_USER_LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SYS_USER_LED1_Pin */
  GPIO_InitStruct.Pin = SYS_USER_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYS_USER_LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
static void CAN_FilterConfig(void)
{
  CAN_FilterTypeDef sFilterConfig2;

  /*## Configure the CAN Filter ###########################################*/
  sFilterConfig2.FilterBank = 14; //ID 0-13 for CAN1, 14+ is CAN2
  sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig2.FilterIdHigh = 0x0000;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO1;
  sFilterConfig2.FilterActivation = ENABLE;
  sFilterConfig2.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
}


/**
  * @brief  Rx Fifo 1 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader2, RxData2) == HAL_OK)
  {
    if (RxHeader2.StdId != 0x201)
    {
      CANRxMsg msg;
      switch (RxHeader2.StdId)
      {
      case 0x202:
        msg.can_id = ID_0x202;
        break;
      case 0x203:
        msg.can_id = ID_0x203;
        break;
      case 0x204:
        msg.can_id = ID_0x204;
        break;
      case 0x211:
        msg.can_id = ID_0x211;
        break;
      case 0x212:
        msg.can_id = ID_0x212;
        break;
      case 0x213:
        msg.can_id = ID_0x213;
        break;
      case 0x214:
        msg.can_id = ID_0x214;
        break;
      case 0x215:
        msg.can_id = ID_0x215;
        break;
      case 0x216:
        msg.can_id = ID_0x216;
        break;
      }
      msg.dlc = RxHeader2.DLC;
      memcpy(msg.data, RxData2, RxHeader2.DLC);
      rx_msg_buffer2[buffer_wp2] = msg;
      buffer_wp2++;
      buffer_wp2 %= BUFFER_SIZE;
    }
  }
}

void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader2, RxData2) == HAL_OK)
  {
    if (RxHeader2.StdId != 0x201)
    {
      CANRxMsg msg;
      switch (RxHeader2.StdId)
      {
      case 0x202:
        msg.can_id = ID_0x202;
        break;
      case 0x203:
        msg.can_id = ID_0x203;
        break;
      case 0x204:
        msg.can_id = ID_0x204;
        break;
      case 0x211:
        msg.can_id = ID_0x211;
        break;
      case 0x212:
        msg.can_id = ID_0x212;
        break;
      case 0x213:
        msg.can_id = ID_0x213;
        break;
      case 0x214:
        msg.can_id = ID_0x214;
        break;
      case 0x215:
        msg.can_id = ID_0x215;
        break;
      case 0x216:
        msg.can_id = ID_0x216;
        break;
      }
      msg.dlc = RxHeader2.DLC;
      memcpy(msg.data, RxData2, RxHeader2.DLC);
      rx_msg_buffer2[buffer_wp2] = msg;
      buffer_wp2++;
      buffer_wp2 %= BUFFER_SIZE;
    }
  }
}

















/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  if (htim == &htim2)
  {
    // 1msec Timer
    timer1msec_counter++;
    timer1msec_flag = 1;

    // 10sec conter
    if (timer1msec_counter % 10000 == 0)
    {
      timer10sec_counter++;
      timer10sec_flag = 1;
    }

    // 1sec conter
    if (timer1msec_counter % 1000 == 0)
    {
      timer1sec_counter++;
      timer1sec_flag = 1;
    }

    // 100msec counter
    if (timer1msec_counter % 100 == 0)
    {
      timer100msec_counter++;
      timer100msec_flag = 1;
    }

    // 10msec counter
    if (timer1msec_counter % 10 == 0)
    {
      timer10msec_counter++;
      timer10msec_flag = 1;
    }
  }
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
