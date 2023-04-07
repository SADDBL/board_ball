#include "Control.h"
stepper motor1;
stepper motor2;

pid pid_controler1;
pid pid_controler2;

pid pid_outer_x;
pid pid_outer_y;

int target_step_y = 0;
int target_step_x = 0;

int fabs_int(int val)
{
	if(val>0) return val;
	else return -val;
}

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

//void pid_angle(stepper *motor,int v)
//{
//	motor->Anl_v = v;
//	pid_realize(motor->pid_concroler,)
//}

/********** PID�ײ㺯�� **********/
/* PID��ʼ������ */
void pid_init(pid* pid_controller,float p,float i,float d,PIDOut_Type max,PIDOut_Type min)
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
	pid_controller->max = max;
	pid_controller->min = min;
	pid_controller->output_last = 0;
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

/* λ��ʽPIDʵ�ֺ��� */
//mode��1���ڻ�PID��2���⻷PID
void pid_realize(pid *PID,PIDIn_Type actual_val,int mode)
{
	float switch_d = 1;
	int switch_i = 1;	//���ַ���
	int epsilon_d,epsilon_i;
	int pid_i_max;
	PID->cur_val = actual_val;
	PID->err = PID->target_val - PID->cur_val;
	if(mode==1){//�ڻ�
		epsilon_d = 30;
		epsilon_i = 30;
		pid_i_max = 10000;
	}
	else if(mode==2){//�⻷
		epsilon_d = 25;
		epsilon_i = 20;
		pid_i_max = 4000;
	}
	//�����ֱ���
	if(PID->output_last>PID->max||PID->output_last<PID->min){
		if(PID->output_last*PID->err<0)//errʹ���������ֵ��С
			PID->i += PID->err;
		else PID->i=PID->i;
	}
	else PID->i += PID->err;
	//���ַ����΢�ַ���
	if(fabs_int(PID->err)<epsilon_d) switch_d = 0.3;
	if(fabs_int(PID->err)>epsilon_i) switch_i = 0;
	//�����޷�
	if(PID->i>pid_i_max) PID->i = pid_i_max;
	else if(PID->i<-pid_i_max) PID->i = -pid_i_max;
	PID->output = PID->kp*PID->err + switch_i*PID->ki*PID->i + switch_d*PID->kd*(PID->err - PID->err_k1);
	PID->err_k1 = PID->err;
	PID->output_last = PID->output;
	//����޷�
	if(PID->output>PID->max) PID->output=PID->max;
	if(PID->output<PID->min) PID->output=PID->min;
}

//һ���˲�
#define a 0.8 //�˲�ϵ��
float first_order_filter(float new_value,float last_value)
{
	//a��ȡֵ�������㷨�������ȣ�aԽ���²ɼ���ֵռ��Ȩ��Խ���㷨Խ��������ƽ˳�Բ�
	//�෴��aԽС���²ɼ���ֵռ��Ȩ��ԽС�������Ȳ��ƽ˳�Ժá�
	float flitered = new_value*a + last_value*(1-a);
	return flitered;
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

