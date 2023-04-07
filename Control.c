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

/********** 平板控制函数 **********/
//利用增量式PID控制平板转动指定角度
void pid_dangle(stepper *motor,int v)
{
	int No = motor->No;

	//y轴电机
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

/********** PID底层函数 **********/
/* PID初始化函数 */
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

/* 增量式PID实现函数 */
void pos_pid_realize(pid* PID,PIDIn_Type actual_val)
{
	PID->cur_val = actual_val;
	PID->err = PID->target_val - PID->cur_val;
	//计算de(k)
	PID->output = PID->kp*(PID->err - PID->err_k1) + PID->ki*PID->err + PID->kd*(PID->err + PID->err_k2 - 2*PID->err_k1);
	PID->err_k2=PID->err_k1;
	PID->err_k1=PID->err;
}

/* 位置式PID实现函数 */
//mode：1：内环PID；2：外环PID
void pid_realize(pid *PID,PIDIn_Type actual_val,int mode)
{
	float switch_d = 1;
	int switch_i = 1;	//积分分离
	int epsilon_d,epsilon_i;
	int pid_i_max;
	PID->cur_val = actual_val;
	PID->err = PID->target_val - PID->cur_val;
	if(mode==1){//内环
		epsilon_d = 30;
		epsilon_i = 30;
		pid_i_max = 10000;
	}
	else if(mode==2){//外环
		epsilon_d = 25;
		epsilon_i = 20;
		pid_i_max = 4000;
	}
	//抗积分饱和
	if(PID->output_last>PID->max||PID->output_last<PID->min){
		if(PID->output_last*PID->err<0)//err使积分项绝对值减小
			PID->i += PID->err;
		else PID->i=PID->i;
	}
	else PID->i += PID->err;
	//积分分离和微分分离
	if(fabs_int(PID->err)<epsilon_d) switch_d = 0.3;
	if(fabs_int(PID->err)>epsilon_i) switch_i = 0;
	//积分限幅
	if(PID->i>pid_i_max) PID->i = pid_i_max;
	else if(PID->i<-pid_i_max) PID->i = -pid_i_max;
	PID->output = PID->kp*PID->err + switch_i*PID->ki*PID->i + switch_d*PID->kd*(PID->err - PID->err_k1);
	PID->err_k1 = PID->err;
	PID->output_last = PID->output;
	//输出限幅
	if(PID->output>PID->max) PID->output=PID->max;
	if(PID->output<PID->min) PID->output=PID->min;
}

//一阶滤波
#define a 0.8 //滤波系数
float first_order_filter(float new_value,float last_value)
{
	//a的取值决定了算法的灵敏度，a越大，新采集的值占的权重越大，算法越灵敏，但平顺性差
	//相反，a越小，新采集的值占的权重越小，灵敏度差，但平顺性好。
	float flitered = new_value*a + last_value*(1-a);
	return flitered;
}

/********** 步进电机底层函数 **********/
void stepper_init(stepper* motor,uint16_t stp_pin,GPIO_TypeDef *port,uint16_t dir_pin,uint32_t channel,pid* PID,int No)
{
	motor->No=No;
	motor->Channel = channel;
	motor->Dir_pin=dir_pin;
	motor->Stp_pin=stp_pin;
	motor->Dir_Port = port;
	
	motor->Anl_a = 0;
	motor->Anl_v = 300;
	motor->step_record = 0;	//以初始状态为零点
	motor->target_step = 0;
	
	motor->stepper_running = Stop;
	motor->stepper_dir = Stop;
	
	motor->pid_concroler = PID;
}


/* 设定步进电机需要转过的步数 */
void stepper_ctr(stepper* motor)
{
	//反转
	if(motor->target_step<motor->step_record) {	
		motor->stepper_dir = CCW;
		HAL_GPIO_WritePin(motor->Dir_Port,motor->Dir_pin,GPIO_PIN_SET);	//Dir_pin打高
		HAL_TIM_OC_Start_IT(&htim4,motor->Channel);
		__HAL_TIM_CLEAR_IT(&htim4,motor->Channel);
	}
	//正转
	else if(motor->target_step>motor->step_record){
		motor->stepper_dir = CW;
		HAL_GPIO_WritePin(motor->Dir_Port,motor->Dir_pin,GPIO_PIN_RESET);	//Dir_pin拉低
		HAL_TIM_OC_Start_IT(&htim4,motor->Channel);
		__HAL_TIM_CLEAR_IT(&htim4,motor->Channel);
	}
	//开启输出比较中断
}

