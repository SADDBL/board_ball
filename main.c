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

//调零角度
#define a_x_0 (float) 1.1
#define a_y_0 (float) -1.4

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,10);
	return ch;

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
}

void delay_us(uint32_t us);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

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
void init_main(void);
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

int OC_Channel1_Pulse,OC_Channel2_Pulse;//输出比较Pulse值，决定输出频率，f=1MHz/Pulse
int OC_Channel1_Duty,OC_Channel2_Duty;//输出比较Duty值，决定占空比，即Duty%

//矩阵键盘输入数值
int keyborad_data;

int x0 = 336;
int y0 = 333;

//小球目标位置(x,y)
int x_target = 336,y_target = 333;

//小球当前位置(x,y)
int x_cur = -1,y_cur = -1;

//PID控制器的参数，写成全局方便修改
//float kp1=2,ki1=0.01,kd1=16;
//float kp2=2,ki2=0.01,kd2=16;
float kp1=5.5,ki1=0,kd1=0;
float kp2=5.5,ki2=0,kd2=0;
//PID外环
float k_outer_p1=0.4,k_outer_i1=0.004,k_outer_d1=4;
float k_outer_p2=0.4,k_outer_i2=0.004,k_outer_d2=4;
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
	OC_Channel1_Duty=50;
	OC_Channel2_Duty=50;
	
	//PID控制器结构体初始化
	pid_init(&pid_controler1,kp1,ki1,kd1,350,-350);
	pid_init(&pid_controler2,kp2,ki2,kd2,350,-350);
	pid_init(&pid_outer_y,k_outer_p1,k_outer_i1,k_outer_d1,300,-300);
	pid_init(&pid_outer_x,k_outer_p1,k_outer_i2,k_outer_d2,300,-300);
	
	//步进电机结构体初始化
	stepper_init(&motor1,GPIO_PIN_6,GPIOA,GPIO_PIN_15,TIM_CHANNEL_1,&pid_controler1,1);
	stepper_init(&motor2,GPIO_PIN_7,GPIOB,GPIO_PIN_3,TIM_CHANNEL_2,&pid_controler2,2);
	
	//主函数初始化
	init_main();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/***** OLED操作 *****/
		//main界面
		if(MENU==MAIN_MENU){
			oled_main_menu_opera();
		}
		//set界面
		else if(MENU==SET_MENU){
			oled_set_menu_opera();
		}
		//task界面
		else if(MENU==TASK_MENU){
			oled_task_menu_opera();
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
		if(data!=0){
			keyborad_data=data;
			printf("%d\r\n",keyborad_data);
		}
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
	}
}

int task_flag = 0;//flag = 1，小球在指定范围内，count计数；flag = 0，小球不在指定范围内，count = 0
int task_count = 0;
//定时器中断函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int stepper_No_last,stepper_angle_last;
	static int x_last = 0, y_last = 0;
	static int v_x_last,v_y_last;
	int v_x=0,v_y=0;
  
	/***** TIM1-定时器中断-50Hz/20ms	*****/
	if(htim->Instance == TIM1){
		/***** 任务函数 *****/
		if(task==1) task1();
		else if(task==2) task2();
		else if(task==3) task3();
		else if(task==4) task4();
		
		//计时
		if(task_flag == 0){
			task_count = 0;
		}
		else{
			task_count++;
		}
		
		/***** PID控制 *****/
		if(x_cur==-1&&y_cur==-1){
			x_last=x0;
			y_last=y0;
		}
		else if(x_cur!=-1&&y_cur!=-1){
			/* 对摄像头数据进行滤波 */
			if(x_cur==0) x_cur = x_last;
			if(y_cur==0) y_cur = y_last;
			x_cur = first_order_filter(x_cur,x_last,0.8);
			y_cur = first_order_filter(y_cur,y_last,0.8);
			if(x_last!=x_cur){
				/* 外环PID */
				//输入：位置坐标
				//输出：小球目标速度值
				pid_outer_x.target_val = x_target;
				pid_realize(&pid_outer_x,x_cur,2);
				/* 内环PID */
				//输入：小球目标速度值
				//输出：步进电机目标脉冲数
				motor2.pid_concroler->target_val = pid_outer_x.output;
				v_x=x_cur-x_last;
	//			v_x=first_order_filter(v_x,v_x_last);
				v_x_last=v_x;
				pid_realize(motor2.pid_concroler,v_x,1);
				first_order_filter(motor2.pid_concroler->output,motor2.pid_concroler->output_last,0.5);
				motor2.target_step = motor2.pid_concroler->output;
				motor2.Anl_v = 200;
				x_last=x_cur;
				stepper_ctr(&motor2);
			}
			if(y_last!=y_cur){
				/* 外环PID */
				//输入：位置坐标
				//输出：小球目标速度值
				pid_outer_y.target_val = y_target;
				pid_realize(&pid_outer_y,y_cur,2);
				/* 内环PID */
				//输入：小球目标速度值
				//输出：步进电机目标脉冲数
				motor1.pid_concroler->target_val = pid_outer_y.output;
				v_y=y_cur-y_last;
	//			v_y=first_order_filter(v_y,v_y_last);
				v_y_last=v_y;
				pid_realize(motor1.pid_concroler,v_y,1);
				first_order_filter(motor1.pid_concroler->output,motor1.pid_concroler->output_last,0.5);
				motor1.target_step = motor1.pid_concroler->output;
				motor1.Anl_v = 200;
				y_last=y_cur;
				//printf("e1=%f,e2=%f,e3=%f\r\n",motor1.pid_concroler->err,motor1.pid_concroler->err_k1,motor1.pid_concroler->err_k2);
				stepper_ctr(&motor1);
			}
		}
	}
	/***** 串口控制 *****/
		if(stepper_angle_last==stepper_usart_angle[1]&&(stepper_No_last==stepper_usart_angle[0]));
		else{
			switch((int)stepper_usart_angle[0]){
				case 1:
				motor1.target_step = (stepper_usart_angle[1]/MICRO_STEP_ANGLE);
				stepper_angle_last = stepper_usart_angle[1];
				stepper_No_last = stepper_usart_angle[0];
				stepper_ctr(&motor1);
				break;
				case 2:
				motor2.target_step = (stepper_usart_angle[1]/MICRO_STEP_ANGLE);
				stepper_angle_last = stepper_usart_angle[1];
				stepper_No_last = stepper_usart_angle[0];
				stepper_ctr(&motor2);
				break;
				default: break;
			}
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

/* 显示参数 */
/**
 * @brief  主函数初始化
 */
void init_main(void){
	int i;
	//任务点列表初始化
//	for(i = 0;i<5;i++){
//		point_list[i].x = 0;
//		point_list[i].y = 0;
//	}
	point_list[0].x = 336;
	point_list[0].y = 460;
	
	point_list[1].x = 210;
	point_list[1].y = 337;
	
	point_list[2].x = 460;
	point_list[2].y = 333;
	
	point_list[3].x = 336;
	point_list[3].y = 200;
	
	point_list[4].x = 336;
	point_list[4].y = 333;
	
	task = 0;
	x_cur = -1;
	y_cur = -1;
	
	//平板调零
	motor1.target_step = a_x_0/MICRO_STEP_ANGLE;
	motor2.target_step = a_y_0/MICRO_STEP_ANGLE;
	stepper_ctr(&motor1);
	HAL_Delay(50);
	stepper_ctr(&motor2);
	HAL_Delay(200);
	motor1.target_step = 0;
	motor1.step_record = 0;
	motor2.target_step = 0;
	motor2.step_record = 0;
	HAL_Delay(200);
	
	//界面相关初始化
	MENU = MAIN_MENU;
}

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
