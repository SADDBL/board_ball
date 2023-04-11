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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Control.h"
#include "connect.h"
#include "oled.h"
#include "Hardware.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define x0 340
#define y0 340

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,10);
	return ch;
}

void delay_us(uint32_t us);
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
extern int target_step_y;

int OC_Channel1_Pulse,OC_Channel2_Pulse;//输出比较Pulse值，决定输出频率，f=1MHz/Pulse
int OC_Channel1_Duty,OC_Channel2_Duty;//输出比较Duty值，决定占空比，即Duty%

//矩阵键盘输入数值
int keyborad_data;

//小球目标位置(x,y)
int x_target = x0,y_target = y0;

//小球当前位置(x,y)
int x_cur = x0,y_cur = y0;

//PID控制器的参数，写成全局方便修改
float kp1=1.5,ki1=0,kd1=0;
float kp2=1.5,ki2=0,kd2=0;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2,RecieveBuffer,1);
	HAL_UART_Receive_IT(&huart3,RecieveBuffer3,1);
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE );
	HAL_TIM_Base_Start_IT(&htim1);
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
	OLED_Init();
	OLED_DisPlay_On();
	oled_pid_para_dis(1);
	OC_Channel1_Duty=50;
	OC_Channel2_Duty=50;
	
	//PID控制器结构体初始化
	pid_init(&pid_controler1,kp1,ki1,kd1);
	pid_init(&pid_controler2,kp2,ki2,kd2);
	
	//步进电机结构体初始化
	stepper_init(&motor1,GPIO_PIN_6,GPIOA,GPIO_PIN_15,TIM_CHANNEL_1,&pid_controler1,1);
	stepper_init(&motor2,GPIO_PIN_7,GPIOB,GPIO_PIN_3,TIM_CHANNEL_2,&pid_controler2,2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		printf("ycur=%0.f,yex=%d,out=%f\r\n",motor1.pid_concroler->cur_val,target_step_y,motor1.pid_concroler->output);
		//printf("y_cur=%d,e1=%f,e2=%f,e3=%f\r\n",y_cur,motor1.pid_concroler->err,motor1.pid_concroler->err_k1,motor1.pid_concroler->err_k2);
		HAL_Delay(7);
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

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_7){
		int i,data=0;
		LDOut_High;
		delay_us(100);
		LDOut_Low;
		for(i=0;i<16;i++){
			delay_us(1);
			if(DAin==GPIO_PIN_SET) data = 16 - i;
			CKOut_High;
			delay_us(1);
			CKOut_Low;
		}
		keyborad_data=data;
//		printf("%d\r\n",keyborad_data);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
	}
}

//定时器中断函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int stepper_No_last,stepper_angle_last;
	static int x_last = x0, y_last = y0;
  
	/***** TIM1-定时器中断-50Hz *****/
	if(htim->Instance == TIM1){
		/***** PID控制 *****/
		if(x_last!=x_cur){
			pid_dangle(&motor2,500);
			x_last=x_cur;
			stepper_ctr(&motor2);
		}
		if(y_last!=y_cur){
			pid_dangle(&motor1,500);
			y_last=y_cur;
			//printf("e1=%f,e2=%f,e3=%f\r\n",motor1.pid_concroler->err,motor1.pid_concroler->err_k1,motor1.pid_concroler->err_k2);
			stepper_ctr(&motor1);
		}
			//pid_dangle(&motor2,225);		
			//pid_dangle(&motor1,225);
			//x_last=x_cur;
			//y_last=y_cur;
			//stepper_ctr(&motor2);
  		//stepper_ctr(&motor1);
	}
}

//输出比较中断函数
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t OC_Count = 0;
	
	/***** TIM4-输出比较 *****/
  if(htim->Instance == TIM4)
  {
		OC_Count = __HAL_TIM_GET_COUNTER(htim);
		
		//No2 电机
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
			
      if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,motor2.Stp_pin))
      {
				if(motor2.target_step==motor2.step_record)
				{
					HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_2);
				}
				OC_Channel2_Pulse = (1000000*MICRO_STEP_ANGLE)/motor2.Anl_v;//OC_Channel1_Pulse每个周期时钟个数，OC_Channel1_Duty%每个周期高电平时钟个数占比
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,OC_Count + OC_Channel2_Pulse - OC_Channel2_Duty*OC_Channel2_Pulse/100);//算式计算的是上升沿到来的时间节点
      }
      else
      {
				if(motor2.target_step>motor2.step_record)//正转
				{
					motor2.step_record++;
				}
				else if(motor2.target_step<motor2.step_record)//反转
				{
					motor2.step_record--;
				}
				OC_Channel2_Pulse = (1000000*MICRO_STEP_ANGLE)/motor2.Anl_v;
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,OC_Count + OC_Channel2_Duty*OC_Channel2_Pulse/100);//算式计算的是下降沿到来的时间节点
      }
    }
		//No1 电机
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
			
      if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,motor1.Stp_pin))
      {
				if(motor1.target_step==motor1.step_record)
				{
					HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_1);
				}
				OC_Channel1_Pulse = (1000000*MICRO_STEP_ANGLE)/motor1.Anl_v;//OC_Channel1_Pulse每个周期时钟个数，OC_Channel1_Duty%每个周期高电平时钟个数占比
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,OC_Count + OC_Channel1_Pulse - OC_Channel1_Duty*OC_Channel1_Pulse/100);//算式计算的是上升沿到来的时间节点
      }
      else
      {
				if(motor1.target_step>motor1.step_record)//正转
				{
					motor1.step_record++;
				}
				else if(motor1.target_step<motor1.step_record)//反转
				{
					motor1.step_record--;
				}
				OC_Channel1_Pulse = (1000000*MICRO_STEP_ANGLE)/motor1.Anl_v;
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,OC_Count + OC_Channel1_Duty*OC_Channel1_Pulse/100);//算式计算的是下降沿到来的时间节点
      }
    }
	}
}

//usart3中断函数，使用DMA接收

// 简单延时，仅适用于F1系列
void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
	{
		;
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
