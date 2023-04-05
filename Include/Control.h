#ifndef __CONTROL__
#define __CONTROL__
#include <stdio.h>
#include "main.h"
#include "tim.h"

/* �����Ȧ���� */
#define STEP_ANGLE 1.8f	//��������Ĳ���ǣ���λ����
#define FSRP ((float)(360.0f/STEP_ANGLE))	//ϸ��ǰ���������һȦ����������
	
#define MICRO_STEP 16	//ϸ����
#define MICRO_STEP_ANGLE (STEP_ANGLE/(float)MICRO_STEP)	//ϸ�ֺ󲽾�ǣ���λ���㣬��0.1125��
#define SPR ((int)(FSRP*MICRO_STEP))	//ϸ�ֺ�һȦ����������
	
#define Angle_MAX 45
#define Angle_MIN -45

/* ���״̬ */
#define CW 1	//˳ʱ��ת
#define CCW -1	//��ʱ��ת
#define Running 1	//����ת��
#define Stop 0	//��ֹ

/* PID������ز��� */
typedef int PIDOut_Type;	//PID�����ֵ����������
typedef int PIDIn_Type;	//PID��Ŀ�ꡢ������������

/* PID�ṹ�� */
typedef struct PIDStruct{
	float kp,ki,kd;
	float k1;	//����Ŵ�ϵ��
	PIDIn_Type target_val,cur_val;
	PIDIn_Type err_k2,err_k1,err,i;
	PIDOut_Type max,min;
	PIDOut_Type output;
}pid;

/* ��������ṹ�� */
typedef struct stepmoter{
	int No;	//�����ţ�1��Ӧy������2��Ӧx����
	GPIO_TypeDef *Dir_Port;
	uint16_t Dir_pin;	//��������ź�����
	uint16_t Stp_pin;	//������������
	uint32_t Channel;	//��Ӧ��TIM Channel
	
	/*****��������*****/
	/*****
	��������ʺ�Ӧ���ڵ��ٳ��ϣ�ÿ���ӵ�ת�ٲ�����1000ת
	��ѹ���ת��Ϊ90-900rpm������
	270-2700��/s
	*****/
	int step_record;	//��¼�������ת���Ĳ�����+��ʾ��ת��-��ʾ��ת
	int target_step; //Ŀ�경��
	int Anl_v;	//ת�����ٶȣ���λ����/s
	float Anl_a;	//ת���Ǽ��ٶȣ���λ����/s^2
	
	
	/*****״̬����*****/
	int stepper_dir;	//��־���ת������CWΪ��ת��˳ʱ�룩��CCWΪ��ʱ��
	int stepper_running;	//�������������״̬
	
	pid* pid_concroler;
	
}stepper;

extern stepper motor1;
extern stepper motor2;

extern pid pid_controler1;
extern pid pid_controler2;

int fabs_int(int val);
/********** PID�ײ㺯�� **********/
void pid_init(pid* pid_controller,float p,float i,float d,PIDOut_Type max,PIDOut_Type min);
void pos_pid_realize(pid* PID,PIDIn_Type actual_val);
void pid_realize(pid* PID,PIDIn_Type actual_val);

/********** ����ײ㺯�� **********/
void stepper_init(stepper* motor,uint16_t stp_pin,GPIO_TypeDef *port,uint16_t dir_pin,uint32_t channel,pid* PID,int No);
void stepper_ctr(stepper* motor);

/********** ƽ����ƺ��� **********/
void pid_dangle(stepper *motor,int v);

#endif
