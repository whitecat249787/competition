/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_database.h"
#include "com_chassis.h"
#include "pid.h"
#include "dji_motor.h"
#include "basic_action.h"
#include "stdio.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int c,FILE *stream)
{
  uint8_t ch[1]={c};
  HAL_UART_Transmit(&huart7,ch,1,0XFFFF);
  return c;
}

uint8_t way[1];//ͨ������1��2��3��4������������������ĸ�����Ĳ���
uint32_t time[4]={0};
uint8_t xyz[3];
uint32_t remote_time;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_UART8_Init();
  MX_USART6_UART_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    Can_start_work();
	Pid_parameter_init();
	//HAL_UART_Receive_IT(&huart7, &recv_data, 1);
    //__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	time[0]=HAL_GetTick();
	time[1]=time[0];time[2]=time[0];time[3]=time[0];
    printf("OK_1\n");
    HAL_UART_Receive_IT(&huart7,way,1);//ͨ������1��2��3��4������������������ĸ�����Ĳ���
  while (1)
  {
    	time[0]=HAL_GetTick();
      
//     CAN_TxHeaderTypeDef  last_four_motor_tx_message;
//uint8_t              last_four_motor_can_send_data[8]={0};
//    uint32_t send_mail_box;
//    last_four_motor_tx_message.StdId = 0x101;
//    last_four_motor_tx_message.IDE = CAN_ID_STD;
//    last_four_motor_tx_message.RTR = CAN_RTR_DATA;
//    last_four_motor_tx_message.DLC = 0x08;
//    HAL_CAN_AddTxMessage(&hcan1, &last_four_motor_tx_message, last_four_motor_can_send_data, &send_mail_box);//2

//HAL_Delay(100);
      
      //����ģʽ��ʼִ������֮ǰ���ǵý��������total_angle��ֵ��Ϊ0����������������������������������������
			//time[1]=time[0];
		//}
//		if(time[0]-time[2]>100){
//				USART_printf("%d\n",Get_dji_information(1).total_angle);
//			time[2]=time[0];
//		}
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 90;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//�ûص�������������debug.c(�ѱ�if 0);

#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==UART7)
    {
        HAL_UART_Receive_IT(&huart7,way,1);//����������ݶ�λ��ģʽ�����У�ģʽ����λ��basic_action.c;
    }
//    if(huart->Instance==USART1)
//    {
//        remote_time=HAL_GetTick();
//        ////////////////////get_real(xyz);//////////�˴����뺯���������x,y,z������д����õ��ܹ���Ӧ��ǰ��е��λ�õ����꣬���崦��������
                                                    
//        HAL_UART_Receive_DMA(&huart1,xyz,3);
//    }
}
#endif
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
