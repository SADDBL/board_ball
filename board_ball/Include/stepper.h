#ifndef __STEPPER_H
#define __STEPPER_H
#include "main.h"
#include "tim.h"
#include "math.h"

/*
note :
	�ж���λ��ʵ��������������
	����������¸�����ʱʹ�ø������Ľӿڣ�λ�ã��ٶ� �����ٶ� �����ٶ�

��� ʳ�÷�����
	main�г�ʼ��
	eg.Init_Stepper(&Stepper1 , GPIOB , GPIO_PIN_4  , &htim4 , TIM_CHANNEL_3 , 0.1125);
									�ṹ��     ����˿�    ��������   stp��ʱ��		��ʱ��ͨ��      �����
									
									
����λ�õ��ã�Ŀǰ���ٶȺ��ٶȲ���ÿ���ã��ֱ���Ϊ50000 5000�ǱȽϺ��ʵ�
void StpDistanceSetNonBlocking( struct Stepper* stepper , float angdistance , float accel , float tagv );
void StpDistanceSetBlocking( struct Stepper* stepper , float angdistance , float accel , float tagv );


����Ƚ��ж�������������
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim4)------>��ʱ��
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)------>��ʱ��ͨ��
			StepperInOC(&Stepper1);------>��ʱ��ͨ����Ӧ�Ĳ������
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)------>��ʱ��ͨ��
			StepperInOC(&Stepper1);------>��ʱ��ͨ����Ӧ�Ĳ������
	}		
}



*/ 





//------------------------------------------------������������ṹ��------------------------------------------------------//
typedef struct Stepper
{
	uint8_t							CC_FLAG;
	uint8_t 						motor_state;				//���״̬�Ĵ����λ�ֱ��в�ͬ���� \
																						MSB������1cw0ccw��\
																						[X]		Ԥ��	 \
																						[6 :3]  �˶�ģ̬\
																						[2:1]   �˶��׶�\
																						[0]			�˶���ֹ
	uint16_t 						CCR_ValSetter;																						
	uint16_t 						EnPort;							//
	uint16_t						EnPin;							//	
	
	GPIO_TypeDef*				DirPort;						//
	TIM_HandleTypeDef*  StpTim;							//		
	uint16_t    				StpChannel;					//
	uint16_t						DirPin;							//
	uint32_t*   __IO    		ChannelCCR;
	
	uint32_t						period_buffer;
	uint32_t 						period_now;					//��ǰ����
	uint32_t						period_rest;				//���������������µ����ں󱻸�ֵ
	uint32_t 						step_counter;				//��ǰ�׶εĲ�������
	uint32_t 						step_threshold;			//��ǰ�׶ε��ܲ���
	uint32_t						stepff_memory[6];		//������������ [0]���� [1]���� [2]���� 
	
	int									position_ctnow;			//�Լ���������ʽ����ĵ�ǰλ�ã�ת���ɽǶ���Ҫ�˲����


//	float								angv_now;
	float								position_ang;				//�Ը���Ƕȵ���ʽ��ʾ��λ�ã�
	float								stepangle;					//�����
	float								TagAngV;						//Ŀ����ٶ�
	float 							AcceAng;						//�Ǽ��ٶ�
	
	
}Stepper;


void Init_Stepper(Stepper* stp , GPIO_TypeDef* stpdir_gpio_port , uint16_t stpdir_gpio_pin ,\
									TIM_HandleTypeDef* stps_timer , uint16_t stps_pin , float angpp);//��ʼ�������ò�����������ͨ����IO��

//---------------------------------------------------�߼�����----------------------------------------------------\\
void			StopSetBit(uint8_t* flag)       {*flag  &= 0xfe ;}								//ֹͣ

/*
0 0000 00 0
0 : ����
000 0������ƽ�� 000 1������ⶥ 100 0������ƽ�� 100 1������ⶥ

00 �׶�
0  �˶����Ǿ�ֹ

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

#define 		STATETOFULLSTEP(flag)				{(*flag)&=(~TYPEFLAG);(*flag)|= FULLSTEP;}//�л���ֹͣ״̬
#define 		STATETOMIDPOINT(flag)				{(*flag)&=(~TYPEFLAG);(*flag)|= MIDPOINT;}//�л���ֹͣ״̬
#define			DIRCHANGENEEDED(flag)				{(*flag)|=(DIRCHANGEFLAG);}//�л���Ҫ����״̬
#define			DIRCHANGECANCEL(flag)				{(*flag)&=(~DIRCHANGEFLAG);}//�л�������ҪҪ����״̬

#define			MoveSetBit(flag)        {(*flag)&=(~MOVINGFLAG);(*flag)|= MOVINGFLAG;}//�˶�							//
#define			StateToAcc(flag)				{(*flag)&=(~STEPFLAG);(*flag)|= ACCELING;}//�л�������״̬
#define			StateToRun(flag)				{(*flag)&=(~STEPFLAG);(*flag)|= RUNNING;}//�л�������״̬
#define			StateToDec(flag)				{(*flag)&=(~STEPFLAG);(*flag)|= DECELING;}//�л�������״̬
#define			StateToStp(flag)				{(*flag)&=(~STEPFLAG);(*flag)|= STOPING;}//�л���ֹͣ״̬

#define			IFDIRCHANGE(flag)						((flag)&(uint8_t)DIRCHANGEFLAG)
#define			IFMOVING(flag)							((flag)&(uint8_t)MOVINGFLAG)						
#define			GETDIRECT(flag)							((flag)&(uint8_t)DIRFLAG)
#define			GETTYPE(flag)								((flag)&(uint8_t)TYPEFLAG)
#define			GETRUNSTEP(flag)						((flag)&(uint8_t)STEPFLAG)



#define			CW										(uint8_t)0x80
#define			CCW										(uint8_t)0x00
#define			IfCW(flag)						((flag)&(uint8_t)CW)
#define     DIRTOCW(flag)					{(*flag)&=(~DIRFLAG);(*flag)|= CW;} //�л���˳ʱ��״̬
#define 		DIRTOCCW(flag)				{(*flag)&=(~DIRFLAG);(*flag)|= CCW;}//�л�����ʱ��״̬



/*----------------------- ���Ŷ��壬ע��һ��Ҫ����hal�������ú� -----------------------------------*/

/*----------------------- �������ſ��� -----------------------------------*/

/*----------------------- ʧ�����ſ��� -----------------------------------*/
/* ʹ��ʧ�� x = 1 ��Ч��x = 0ʱ��Ч*/                          
                            
                        
/******************************************************************************************/
/* �ⲿ�ӿں���*/
extern struct Stepper Stepper1;

void Init_Stepper(Stepper* stp , GPIO_TypeDef* stpdir_gpio_port , uint16_t stpdir_gpio_pin ,\
									TIM_HandleTypeDef* stps_timer , uint16_t stps_ch , float angpp);


void create_t_ctrl_param(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed , uint32_t stepperchannel); /* ���μӼ��ٿ��ƺ��� */   

void StpDistanceSetNonBlocking( struct Stepper* stepper , float angdistance , float accel , float tagv );
//�����µ��˶�ʱ�����˶��ھ��˶�ͬ���򣬽���Ҫ���ߵĲ�����ӵ���ǰ���˶���

void StpDistanceSetBlocking( struct Stepper* stepper , float angdistance , float accel , float tagv );
//�����µ��˶�ʱ�����˶��ھ��˶�ͬ���򣬽���Ҫ���ߵĲ�����ӵ���ǰ���˶���





#endif