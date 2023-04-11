#include "Control.h"

stepper motor1;
stepper motor2;

pid pid_controler1;
pid pid_controler2;

int target_step_y = 0;
int target_step_x = 0;
/********** ƽ����ƺ��� **********/
//��������ʽPID����ƽ��ת��ָ���Ƕ�
void pid_dangle(stepper *motor,int v)
{
	int No = motor->No;

	//y����
	if(No==1){
		motor->pid_concroler->target_val = y_target;
		pos_pid_realize(motor->pid_concroler,y_cur);
		target_step_y += motor->pid_concroler->output;
		motor->target_step = target_step_y;
		if(motor->target_step>=350)
			motor->target_step=350;
		else if(motor->target_step<=-350)
			motor->target_step=-350;
		motor->Anl_v = v;
	}
	if(No==2){
		motor->pid_concroler->target_val = x_target;
		pos_pid_realize(motor->pid_concroler,x_cur);
		target_step_x += motor->pid_concroler->output;
		motor->target_step = target_step_x;
		if(motor->target_step>=350)
			motor->target_step=350;
		else if(motor->target_step<=-350)
			motor->target_step=-350;
		motor->Anl_v = v;
	}
}

/********** PID�ײ㺯�� **********/
/* PID��ʼ������ */
void pid_init(pid* pid_controller,float p,float i,float d)
{
	pid_controller->kp = p;
	pid_controller->ki = i;
	pid_controller->kd = d;
	pid_controller->cur_val = 0;
	pid_controller->target_val = 0;
	pid_controller->err = 0;
	pid_controller->err_k1 = 0;
	pid_controller->err_k2 = 0;
	pid_controller->output = 0;
}

/* ����ʽPIDʵ�ֺ��� */
void pos_pid_realize(pid* PID,PIDIn_Type actual_val)
{
	PID->cur_val = actual_val;
	PID->err = PID->target_val - PID->cur_val;
	//����de(k)
	PID->output = PID->kp*(PID->err - PID->err_k1) + PID->ki*PID->err + PID->kd*(PID->err + PID->err_k2 - 2*PID->err_k1);
	PID->err_k2=PID->err_k1;
	PID->err_k1=PID->err;
}


/********** ��������ײ㺯�� **********/
void stepper_init(stepper* motor,uint16_t stp_pin,GPIO_TypeDef *port,uint16_t dir_pin,uint32_t channel,pid* PID,int No)
{
	motor->No=No;
	motor->Channel = channel;
	motor->Dir_pin=dir_pin;
	motor->Stp_pin=stp_pin;
	motor->Dir_Port = port;
	
	motor->Anl_a = 0;
	motor->Anl_v = 300;
	motor->step_record = 0;	//�Գ�ʼ״̬Ϊ���
	motor->target_step = 0;
	
	motor->stepper_running = Stop;
	motor->stepper_dir = Stop;
	
	motor->pid_concroler = PID;
}


/* �趨���������Ҫת���Ĳ��� */
void stepper_ctr(stepper* motor)
{
	//��ת
	if(motor->target_step<motor->step_record) {	
		motor->stepper_dir = CCW;
		HAL_GPIO_WritePin(motor->Dir_Port,motor->Dir_pin,GPIO_PIN_SET);	//Dir_pin���
		HAL_TIM_OC_Start_IT(&htim4,motor->Channel);
		__HAL_TIM_CLEAR_IT(&htim4,motor->Channel);
	}
	//��ת
	else if(motor->target_step>motor->step_record){
		motor->stepper_dir = CW;
		HAL_GPIO_WritePin(motor->Dir_Port,motor->Dir_pin,GPIO_PIN_RESET);	//Dir_pin����
		HAL_TIM_OC_Start_IT(&htim4,motor->Channel);
		__HAL_TIM_CLEAR_IT(&htim4,motor->Channel);
	}
	//��������Ƚ��ж�
	
}

