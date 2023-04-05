#ifndef __CONTROL__
#define __CONTROL__
#include <stdio.h>
#include "main.h"
#include "tim.h"

/* 电机单圈参数 */
#define STEP_ANGLE 1.8f	//步进电机的步距角，单位：°
#define FSRP ((float)(360.0f/STEP_ANGLE))	//细分前，步进电机一圈所需脉冲数
	
#define MICRO_STEP 16	//细分数
#define MICRO_STEP_ANGLE (STEP_ANGLE/(float)MICRO_STEP)	//细分后步距角，单位：°，即0.1125°
#define SPR ((int)(FSRP*MICRO_STEP))	//细分后，一圈所需脉冲数
	
#define Angle_MAX 45
#define Angle_MIN -45

/* 电机状态 */
#define CW 1	//顺时针转
#define CCW -1	//逆时针转
#define Running 1	//正在转动
#define Stop 0	//静止

/* PID控制相关参数 */
typedef int PIDOut_Type;	//PID的输出值的数据类型
typedef int PIDIn_Type;	//PID的目标、误差的数据类型

/* PID结构体 */
typedef struct PIDStruct{
	float kp,ki,kd;
	float k1;	//增益放大系数
	PIDIn_Type target_val,cur_val;
	PIDIn_Type err_k2,err_k1,err,i;
	PIDOut_Type max,min;
	PIDOut_Type output;
}pid;

/* 步进电机结构体 */
typedef struct stepmoter{
	int No;	//电机编号，1对应y轴电机，2对应x轴电机
	GPIO_TypeDef *Dir_Port;
	uint16_t Dir_pin;	//方向控制信号引脚
	uint16_t Stp_pin;	//脉冲输入引脚
	uint32_t Channel;	//对应的TIM Channel
	
	/*****参数变量*****/
	/*****
	步进电机适合应用在低速场合，每分钟的转速不超过1000转
	最佳工作转速为90-900rpm，即：
	270-2700°/s
	*****/
	int step_record;	//记录从零点起转过的步数，+表示正转，-表示反转
	int target_step; //目标步数
	int Anl_v;	//转动角速度，单位：°/s
	float Anl_a;	//转动角加速度，单位：°/s^2
	
	
	/*****状态变量*****/
	int stepper_dir;	//标志电机转动方向，CW为正转（顺时针），CCW为逆时针
	int stepper_running;	//步进电机的运行状态
	
	pid* pid_concroler;
	
}stepper;

extern stepper motor1;
extern stepper motor2;

extern pid pid_controler1;
extern pid pid_controler2;

int fabs_int(int val);
/********** PID底层函数 **********/
void pid_init(pid* pid_controller,float p,float i,float d,PIDOut_Type max,PIDOut_Type min);
void pos_pid_realize(pid* PID,PIDIn_Type actual_val);
void pid_realize(pid* PID,PIDIn_Type actual_val);

/********** 电机底层函数 **********/
void stepper_init(stepper* motor,uint16_t stp_pin,GPIO_TypeDef *port,uint16_t dir_pin,uint32_t channel,pid* PID,int No);
void stepper_ctr(stepper* motor);

/********** 平板控制函数 **********/
void pid_dangle(stepper *motor,int v);

#endif
