/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define INP_VOL 2.0
#define STEP 400
//#define limit 500
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

float frac = INP_VOL/3.0 ;
float vol=0.0;
float currentl=0.0;
float currentr=0.0;
uint32_t i = 1000, vall = 0,valr = 0;
CAN_TxHeaderTypeDef Txheader;
CAN_RxHeaderTypeDef Rxheader;
uint8_t Txmsg[8];
uint8_t Rxmsg[8];
uint8_t uartbuf[32];
uint8_t uartbufhere[2];
signed int deltal=0;
signed int deltar=0;
uint8_t speedlsbl=0;
uint8_t speedmsbl=0;
uint16_t speedl=0;
uint8_t speedlsbr=0;
uint8_t speedmsbr=0;
uint16_t speedr=0;
uint8_t currentmsbl =0;
uint8_t currentlsbl =0;
uint8_t currentmsbr =0;
uint8_t currentlsbr =0;
int speeddif =0;
signed int limit=0;

uint32_t TxMailbox;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_DAC_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
static void MX_CAN_Message_Init(void);
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
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  MX_CAN_Message_Init();
  HAL_CAN_StateTypeDef status ;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1);
	  HAL_UART_Receive_IT(&huart5, uartbuf, 32);
	  status  = HAL_CAN_GetState(&hcan1);
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R , vall);
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R , valr);
	  speeddif = sqrt((speedr-speedl)*(speedr-speedl));


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
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hcan1.Init.Prescaler = 36;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef sFilter;
    sFilter.FilterBank = 0;
    sFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilter.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilter.FilterIdHigh = 0x0000; //((ID >> 13) & 0x001F);
    sFilter.FilterIdLow = 0x0000;//((ID << 3) & 0xFFF8);
    sFilter.FilterMaskIdHigh = 0x0000;
    sFilter.FilterMaskIdLow = 0x0000;
    sFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0 ;
    sFilter.FilterActivation = ENABLE;
    sFilter.SlaveStartFilterBank = 0 ;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilter) != HAL_OK)
     {
       Error_Handler();
     }
    if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)!= HAL_OK)
    {
  	  Error_Handler();
    }
    if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY)!= HAL_OK)
    {
  	  Error_Handler();
    }
   if(HAL_CAN_Start(&hcan1)!= HAL_OK)
    {
    	 Error_Handler();
    }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */
  HAL_DAC_Start(&hdac,DAC_CHANNEL_1 );
  HAL_DAC_Start(&hdac,DAC_CHANNEL_2 );
  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
static void MX_CAN_Message_Init(void)
{
	Txheader.DLC = 8;
	Txheader.IDE = CAN_ID_EXT;
	Txheader.RTR = CAN_RTR_DATA;
	Txheader.StdId = 0x11;
	Txheader.ExtId = 0x10000004;
	Txheader.TransmitGlobalTime = DISABLE ;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_FILTER_FIFO0, &Rxheader, Rxmsg);


	if(Rxheader.ExtId==0x0CF11E06)
	{
	  speedlsbl=Rxmsg[0] ;
	  speedmsbl=Rxmsg[1] ;
	  currentlsbl =Rxmsg[2];
	  currentmsbl =Rxmsg[3];
	  speedl = (256)*speedmsbl + speedlsbl;
	  //currentl= (256*currentmsbl+currentlsbl)/10.0;
	  if(limit<5)
	  {
		vall=0;
	  }
	  else
	  {
	  if(speedl==limit-deltal)
	  {
		  ;//vall=vall;
	  }
	  else
		  {
		  if(speedl > limit-deltal)
		  {
			  if(vall>2000)
			  {
				vall=2000;
			  }
			  else
			  {
			  vall=vall-1;
			  }
		  }
		  else
		  {
			  if(vall<1300)
			  {
				  vall=1300;
			  }
			  else
			  {
			  vall=vall+5;
			  }
		  }
		}
	}
	}
	if(Rxheader.ExtId==0x0CF11E05)
	{
	  speedlsbr=Rxmsg[0] ;
	  speedmsbr=Rxmsg[1] ;
	  currentlsbr =Rxmsg[2];
	  currentmsbr =Rxmsg[3];
	  speedr = (256)*speedmsbr + (speedlsbr);
	  //currentr= (256*currentmsbr+currentlsbr)/10.0;
	  if(limit<5)
	  {
		  valr=0;
	  }
	  else
	  {
	  if(speedr==limit+deltar)
	  {
		  ;//valr=valr;
	  }
	  else
		  {
		  if(speedr > limit+deltar)
		  {
			  if(valr>2000)
			  {
				valr=2000;
			  }
			  else
			  {
			  valr=valr-1;
			  }
		  }
		  else
		  {
			  if(valr<1300)
			  {
				  valr=1300;
			  }
			  else
			  {
			  valr=valr+5;
			  }
		  }
		}
	  }

	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
if(1);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uartbufhere[0]=uartbuf[6];
	uartbufhere[1]=uartbuf[7];
	limit= 0.5*((uartbufhere[1]<<8 | uartbufhere[0]) - 1000);
	if(limit>500) limit =500;
	if(limit<0)limit=0;
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

