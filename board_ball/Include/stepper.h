#ifndef __STEPPER_H
#define __STEPPER_H
#include "main.h"
#include "tim.h"
#include "math.h"

/*
note :
	中断中位置实际用整形来处理
	带入计算重新赋参数时使用浮点数的接口，位置，速度 ，加速度 ，减速度

裸机 食用方法：
	main中初始化
	eg.Init_Stepper(&Stepper1 , GPIOB , GPIO_PIN_4  , &htim4 , TIM_CHANNEL_3 , 0.1125);
									结构体     方向端口    方向引脚   stp定时器		定时器通道      步距角
									
									
任意位置调用，目前加速度和速度参数每调好，分别设为50000 5000是比较合适的
void StpDistanceSetNonBlocking( struct Stepper* stepper , float angdistance , float accel , float tagv );
void StpDistanceSetBlocking( struct Stepper* stepper , float angdistance , float accel , float tagv );


输出比较中断中添加如下语句
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim4)------>定时器
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)------>定时器通道
			StepperInOC(&Stepper1);------>定时器通道对应的步进电机
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)------>定时器通道
			StepperInOC(&Stepper1);------>定时器通道对应的步进电机
	}		
}



*/ 





//------------------------------------------------步进电机参数结构体------------------------------------------------------//
typedef struct Stepper
{
	uint8_t							CC_FLAG;
	uint8_t 						motor_state;				//电机状态寄存各个位分别有不同意义 \
																						MSB（方向1cw0ccw）\
																						[X]		预留	 \
																						[6 :3]  运动模态\
																						[2:1]   运动阶段\
																						[0]			运动或静止
	uint16_t 						CCR_ValSetter;																						
	uint16_t 						EnPort;							//
	uint16_t						EnPin;							//	
	
	GPIO_TypeDef*				DirPort;						//
	TIM_HandleTypeDef*  StpTim;							//		
	uint16_t    				StpChannel;					//
	uint16_t						DirPin;							//
	uint32_t*   __IO    		ChannelCCR;
	
	uint32_t						period_buffer;
	uint32_t 						period_now;					//当前周期
	uint32_t						period_rest;				//余数，在运算完新的周期后被赋值
	uint32_t 						step_counter;				//当前阶段的步数计数
	uint32_t 						step_threshold;			//当前阶段的总步数
	uint32_t						stepff_memory[6];		//计算各个步骤点 [0]加速 [1]匀速 [2]减速 
	
	int									position_ctnow;			//以计数器的形式计算的当前位置，转换成角度需要乘步距角


//	float								angv_now;
	float								position_ang;				//以浮点角度的形式表示的位置，
	float								stepangle;					//步距角
	float								TagAngV;						//目标角速度
	float 							AcceAng;						//角加速度
	
	
}Stepper;


void Init_Stepper(Stepper* stp , GPIO_TypeDef* stpdir_gpio_port , uint16_t stpdir_gpio_pin ,\
									TIM_HandleTypeDef* stps_timer , uint16_t stps_pin , float angpp);//初始化是配置步进电机的相关通道与IO等

//---------------------------------------------------逻辑部分----------------------------------------------------\\
void			StopSetBit(uint8_t* flag)       {*flag  &= 0xfe ;}								//停止

/*
0 0000 00 0
0 : 方向
000 0：单向平顶 000 1：单向尖顶 100 0：变向平顶 100 1：变向尖顶

00 阶段
0  运动还是静止

*/
#define			MOVINGFLAG						(uint8_t)0x01						
#define			DIRFLAG					 			(uint8_t)0x80
#define			TYPEFLAG						  (uint8_t)0x08
#define			STEPFLAG						 	(uint8_t)0x06
#define			DIRCHANGEFLAG					(uint8_t)0x40

#define			FULLSTEP							(uint8_t)0x00
#define			MIDPOINT							(uint8_t)0x08
//#define			STATESINGLE						(uint8_t)0x18


#define			ACCELING							(uint8_t)0x02
#define			RUNNING								(uint8_t)0x04
#define			DECELING							(uint8_t)0x06
#define			STOPING								(uint8_t)0x00		

#define 		STATETOFULLSTEP(flag)				{(*flag)&=(~TYPEFLAG);(*flag)|= FULLSTEP;}//切换到停止状态
#define 		STATETOMIDPOINT(flag)				{(*flag)&=(~TYPEFLAG);(*flag)|= MIDPOINT;}//切换到停止状态
#define			DIRCHANGENEEDED(flag)				{(*flag)|=(DIRCHANGEFLAG);}//切换到要变向状态
#define			DIRCHANGECANCEL(flag)				{(*flag)&=(~DIRCHANGEFLAG);}//切换到不需要要变向状态

#define			MoveSetBit(flag)        {(*flag)&=(~MOVINGFLAG);(*flag)|= MOVINGFLAG;}//运动							//
#define			StateToAcc(flag)				{(*flag)&=(~STEPFLAG);(*flag)|= ACCELING;}//切换到加速状态
#define			StateToRun(flag)				{(*flag)&=(~STEPFLAG);(*flag)|= RUNNING;}//切换到匀速状态
#define			StateToDec(flag)				{(*flag)&=(~STEPFLAG);(*flag)|= DECELING;}//切换到减速状态
#define			StateToStp(flag)				{(*flag)&=(~STEPFLAG);(*flag)|= STOPING;}//切换到停止状态

#define			IFDIRCHANGE(flag)						((flag)&(uint8_t)DIRCHANGEFLAG)
#define			IFMOVING(flag)							((flag)&(uint8_t)MOVINGFLAG)						
#define			GETDIRECT(flag)							((flag)&(uint8_t)DIRFLAG)
#define			GETTYPE(flag)								((flag)&(uint8_t)TYPEFLAG)
#define			GETRUNSTEP(flag)						((flag)&(uint8_t)STEPFLAG)



#define			CW										(uint8_t)0x80
#define			CCW										(uint8_t)0x00
#define			IfCW(flag)						((flag)&(uint8_t)CW)
#define     DIRTOCW(flag)					{(*flag)&=(~DIRFLAG);(*flag)|= CW;} //切换到顺时针状态
#define 		DIRTOCCW(flag)				{(*flag)&=(~DIRFLAG);(*flag)|= CCW;}//切换到逆时针状态



/*----------------------- 引脚定义，注意一定要先在hal库里配置好 -----------------------------------*/

/*----------------------- 方向引脚控制 -----------------------------------*/

/*----------------------- 失能引脚控制 -----------------------------------*/
/* 使能失能 x = 1 有效，x = 0时无效*/                          
                            
                        
/******************************************************************************************/
/* 外部接口函数*/
extern struct Stepper Stepper1;

void Init_Stepper(Stepper* stp , GPIO_TypeDef* stpdir_gpio_port , uint16_t stpdir_gpio_pin ,\
									TIM_HandleTypeDef* stps_timer , uint16_t stps_ch , float angpp);


void create_t_ctrl_param(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed , uint32_t stepperchannel); /* 梯形加减速控制函数 */   

void StpDistanceSetNonBlocking( struct Stepper* stepper , float angdistance , float accel , float tagv );
//设置新的运动时，新运动于旧运动同方向，将需要多走的步数添加到当前的运动上

void StpDistanceSetBlocking( struct Stepper* stepper , float angdistance , float accel , float tagv );
//设置新的运动时，新运动于旧运动同方向，将需要多走的步数添加到当前的运动上





#endif