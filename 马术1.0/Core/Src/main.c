/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "rc_dog.h"

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//	extern CAN_HandleTypeDef hcan1;

	uint8_t buf[3];
	uint8_t lastBuf;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int jumpFlag=0;

//IMU

typedef struct
{
	CAN_RxHeaderTypeDef hdr;
	uint8_t payload[8];
}CAN_RxPacketTypeDef;
float EulerAngle[3];//32位变量

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	can_filter_init();
	HAL_GPIO_WritePin(Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUT_5V_GPIO_Port, OUT_5V_Pin, GPIO_PIN_SET);
	HAL_Delay(800);
//	
	memcpy(StateDetachedParams_Copy,ALLParams,100*StatesMaxNum);//state_detached_params每个元素（DetachedParam型,即每种步态，外加一个u8的ID号）。
																			//实际占据的字节数为4*6*4+4=96+4=100（＋4而不是加1是因为要4字节对齐）。
																			//设定StatesMaxNum，则拷贝的上限为100*StateMaxNum，不要少拷贝，可以多拷贝，但多拷贝的不要用。	
																			//该复制操作不要在任务中进行，而要在操作系统初始化之前进行，否则将给操作系统的运行造成奇怪的问题。
	ImuInit();
	LCD_1in69_test();
	HAL_Delay(40);
	MotorInit();
	HAL_UART_Receive_DMA(&huart4,buf,3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//		Paint_DrawNum(20, 0,rx_data[0],	&Font24, WHITE, BLACK); 
//		Paint_DrawNum(20, 50,rx_data[1],	&Font24, WHITE, BLACK); 
//		Paint_DrawNum(20, 100,rx_data[2],&Font24, WHITE, BLACK); 
//		Paint_DrawNum(20, 150,rx_data[3],&Font24, WHITE, BLACK); 
//		Paint_DrawNum(120, 0,rx_data[4],&Font24, WHITE, BLACK); 
//		Paint_DrawNum(120, 50,rx_data[5],&Font24, WHITE, BLACK); 
//		Paint_DrawNum(120, 100,rx_data[6],&Font24, WHITE, BLACK); 
//		Paint_DrawNum(120, 150,rx_data[7],&Font24, WHITE, BLACK); 
		
//		ExecuteJump();
//		CoupledMoveLeg(time, state_gait_params[1], 0, 1.0,3);
//InitPosture();

//			ChangeKpKd(0.08,0.02);
//			FrontLegCoordinateControl(-LegLenthMin*cos(60*PI/180),LegLenthMin*sin(81*PI/180));
//			BackLegCoordinateControl(-12*sin(30*PI/180),12*cos(30*PI/180));

   	HAL_UART_Receive_DMA(& huart4 ,buf,3);

		if(buf[0] == 97)
		{
			InitPosture();
			jumpFlag=0;
		}
		if(buf[0] == 98)
		{
			TortPosture(0);
		}
		if(buf[0] == 99)
		{
			TortPosture(1.0);
		}
		if(buf[0] == 100)
		{
			RotatePosture(0);
		}
		if(buf[0] == 101)
		{
			RotatePosture(1.0);
		}
		if(buf[0] == 102)
		{
			DogAttitudeControl();
		}
		if(buf[0] == 103)
		{
			ChangeKpKd(1.0,0.08);
			zuo_pingyi();
		}
		if(buf[0] == 104)
		{
			ChangeKpKd(1.0,0.08);
			you_pingyi();
		}
//		if(buf[0] == 105)
//		{
//			if(jumpFlag==0)
//					{
//						Start_Jump(HAL_GetTick());
//						jumpFlag=1;
//					}
//					ExecuteJump();
//		}
//		if(buf[0] == 106)
//		{
//			if(jumpFlag==0)
//					{
//						Start_Jump(HAL_GetTick());
//						jumpFlag=1;
//					}
//					ExecuteJump1();
//		}
//		if(buf[0] == 107)
//		{
//			if(jumpFlag==0)
//					{
//						Start_Jump(HAL_GetTick());
//						jumpFlag=1;
//					}
//					ExecuteJump2();
//		}
//		if(buf[0] == 108)
//		{
//			if(jumpFlag==0)
//					{
//						Start_Jump(HAL_GetTick());
//						jumpFlag=1;
//					}
//					BackFlip();
//		}
//		if(buf[0] == 109)
//		{
//			if(jumpFlag==0)
//					{
//						Start_Jump(HAL_GetTick());
//						jumpFlag=1;
//					}
//					ExecuteJump4();
//		}
//		if(buf[0] == 110)
//		{
//			if(jumpFlag==0)
//					{
//						Start_Jump(HAL_GetTick());
//						jumpFlag=1;
//					}
//					BigJump();
//		}
//		
		
//	IK(1.0);//运动学逆解得到角度
//	theta2=the1/360*6.2832f; 	//角度转换成弧度
//	theta1=the1/360*6.2832f;   //角度转换成弧度
//	//printf("the1=%f\r\n,the2=%f\r\n",omega1+the2,omega2+the1);
//	motor_init1(omega1+theta2);
//	SERVO_Send_recv(&Motor[1], &data[1]);	//将控制指令发送给电机，同时接收返回值
//		SERVO_Send_recv(&Motor[1], &data[1]);	//将控制指令发送给电机，同时接收返回值
////	Data_process2(&data[0]);
////		setting1(0,20,1.0);
////		key();
////		voltage = get_battery_voltage();
////			setting(0,i,1.0,1);
////			i++;
////			HAL_Delay(1000);
////			if(i>25)
////			{
////				i=10;
////			}
////			setting(5,20,1.0,1);
////		CoupledMoveLeg(time,state_gait_params[1],0,-1.0,1);
////		CoupledMoveLeg(time,state_gait_params[1],0.5,1.0,2);
////		CoupledMoveLeg(time,state_gait_params[1],0.5,1.0,3);
////		CoupledMoveLeg(time,state_gait_params[1],0,1.0,4);

//			Paint_DrawFloatNum (150, 150, ALLParams[1].detached_params_0.step_length,3, &Font24, WHITE, BLACK);
//			Paint_DrawFloatNum (150, 200, ALLParams[1].detached_params_1.step_length,3, &Font24, WHITE, BLACK);
//			Paint_DrawFloatNum (0, 150, ALLParams[1].detached_params_0.step_length,3, &Font24, WHITE, BLACK);
//			Paint_DrawFloatNum (0, 200, ALLParams[1].detached_params_1.step_length,3, &Font24, WHITE, BLACK);

//		setting1(70,150);//站立函数一
//		i++;
//		HAL_Delay(100);
//    cmd.W++;
//		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//	    CAN_RxHeaderTypeDef rx_header;
//			uint8_t rx_data[8];

//			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		static CAN_RxPacketTypeDef packet;
    // CAN数据接收
    if (hcan->Instance == hcan1.Instance)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &packet.hdr, packet.payload) == HAL_OK)		//获得接收到的数据头和数据
					{  		
						if(packet.payload[0]== 0x55 &&	packet.payload[1] == 0x53)
						{
							EulerAngle[ROLL]=(int16_t)((int16_t)packet.payload[3] << 8 | packet.payload[2]);
							EulerAngle[PITCH]=(int16_t)((int16_t)packet.payload[5] << 8 | packet.payload[4]);
							EulerAngle[YAW]=(int16_t)((int16_t)packet.payload[7] << 8 | packet.payload[6]);
							
							EulerAngle[ROLL] = EulerAngle[ROLL] / 32768.0f * 180.0f;
							EulerAngle[PITCH] = EulerAngle[PITCH] / 32768.0f * 180.0f;
							EulerAngle[YAW] = EulerAngle[YAW] / 32768.0f * 180.0f;
							
//							Paint_DrawFloatNum (100, 20, EulerAngle[ROLL],3, &Font24, WHITE, BLACK);
//							Paint_DrawFloatNum (100, 70, 	EulerAngle[PITCH] ,3, &Font24, WHITE, BLACK);
//							Paint_DrawFloatNum (100, 120, EulerAngle[YAW],3, &Font24, WHITE, BLACK);
						}	
						
							HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);						// 再次使能FIFO0接收中断
					}
    }
}


void  HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		
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
