#include "Control.h"
stepper motor1;
stepper motor2;

pid pid_controler1;
pid pid_controler2;

pid pid_outer_x;
pid pid_outer_y;

int target_step_y = 0;
int target_step_x = 0;

int task = 0;

Point point_list[5]; 

int fabs_int(int val)
{
	if(val>0) return val;
	else return -val;
}

/********** 平板控制函数 **********/
//任务内容函数:基础任务一
void task1(void){
	x0 = point_list[1].x;
	y0 = point_list[1].y;
	motor1.Anl_v = 100;
	motor2.Anl_v = 100;
	x_target = point_list[1].x+19;
	y_target = point_list[1].y-9;
	pid_outer_x.epsilon_d1 = 25;
	pid_outer_x.epsilon_i1 = 30;
	pid_outer_y.epsilon_d1 = 20;
	pid_outer_y.epsilon_i1 = 15;
	
	pid_outer_x.epsilon_d2 = -20;
	pid_outer_x.epsilon_i2 = -20;
	pid_outer_y.epsilon_d2 = -25;
	pid_outer_y.epsilon_i2 = -20;
	pid_outer_x.i_max = 600;
	pid_outer_y.i_max = 600;
}

//任务内容函数:基础任务二
void task2(void){
	motor1.Anl_v = 300;
	motor2.Anl_v = 300;
	x0 = point_list[0].x;
	y0 = point_list[0].y;
	x_target = point_list[4].x+1;
	y_target = point_list[4].y-5;
	pid_outer_x.epsilon_d1 = 20;
	pid_outer_x.epsilon_i1 = 20;
	pid_outer_y.epsilon_d1 = 20;
	pid_outer_y.epsilon_i1 = 20;
	
	pid_outer_x.epsilon_d2 = -20;
	pid_outer_x.epsilon_i2 = -20;
	pid_outer_y.epsilon_d2 = -20;
	pid_outer_y.epsilon_i2 = -20;
	pid_outer_x.i_max = 800;
	pid_outer_y.i_max = 800;
}

int point_order = 0;

//任务内容函数:基础任务三
void task3(void){
	x0 = point_list[0].x;
	y0 = point_list[0].y;
	
	if(task_count>99){
		if(point_order<1){
			point_order++;
			pid_outer_x.i = 0;
			pid_outer_y.i = 0;
			pid_outer_x.i_max = 500;
		}
	}
	if(point_order==0){
		motor1.Anl_v = 300;
		motor2.Anl_v = 300;
		x_target = point_list[4].x+2;
		y_target = point_list[4].y-3;
		pid_outer_x.epsilon_d1 = 20;
		pid_outer_x.epsilon_i1 = 20;
		pid_outer_y.epsilon_d1 = 20;
		pid_outer_y.epsilon_i1 = 20;
		
		pid_outer_x.epsilon_d2 = -20;
		pid_outer_x.epsilon_i2 = -20;
		pid_outer_y.epsilon_d2 = -20;
		pid_outer_y.epsilon_i2 = -20;
		pid_outer_x.i_max = 900;
		
	}
	else if(point_order == 1){
		
		x_target = point_list[3].x-2;
		y_target = point_list[3].y;
		motor1.Anl_v = 100;
		motor2.Anl_v = 100;
		pid_outer_x.epsilon_d1 = 20;
		pid_outer_x.epsilon_i1 = 20;
		pid_outer_y.epsilon_d1 = 35;
		pid_outer_y.epsilon_i1 = 35;
		
		pid_outer_x.epsilon_d2 = -25;
		pid_outer_x.epsilon_i2 = -28;
		pid_outer_y.epsilon_d2 = 0;
		pid_outer_y.epsilon_i2 = -5;
	}
	task_flag = 0;
	if(fabs_int(point_list[4].x-x_cur)<13&&fabs_int(point_list[4].y-y_cur)<13){
		//float r = 
		task_flag = 1;
	}
}

//任务内容函数:基础任务四
void task4(void){
	x0 = point_list[0].x;
	y0 = point_list[0].y;
	if(point_order==0){
		x_target = 380;
		y_target = 330;
		pid_outer_x.epsilon_d1 = 20;
		pid_outer_x.epsilon_i1 = 20;
		pid_outer_y.epsilon_d1 = 20;
		pid_outer_y.epsilon_i1 = 20;
		
		pid_outer_x.epsilon_d2 = -20;
		pid_outer_x.epsilon_i2 = -20;
		pid_outer_y.epsilon_d2 = -20;
		pid_outer_y.epsilon_i2 = -20;
	}
	else if(point_order == 1){
		x_target = point_list[3].x-2;
		y_target = point_list[3].y;
		motor1.Anl_v = 100;
		motor2.Anl_v = 100;
		pid_outer_x.epsilon_d1 = 20;
		pid_outer_x.epsilon_i1 = 20;
		pid_outer_y.epsilon_d1 = 35;
		pid_outer_y.epsilon_i1 = 35;
		
		pid_outer_x.epsilon_d2 = -25;
		pid_outer_x.epsilon_i2 = -28;
		pid_outer_y.epsilon_d2 = 0;
		pid_outer_y.epsilon_i2 = -5;
	}
	if(fabs_int(y_cur-330)<10){
		//float r = 
		point_order = 1;
		pid_outer_x.i = 0;
		pid_outer_y.i = 0;
		pid_outer_x.i_max = 500;
	}
}

//利用增量式PID控制平板转动指定角度
//void pid_dangle(stepper *motor,int v)
//{
//	int No = motor->No;

//	//y轴电机
//	if(No==1){
//		motor->pid_concroler->target_val = y_target;
//		pos_pid_realize(motor->pid_concroler,y_cur);
//		target_step_y += motor->pid_concroler->output;
//		motor->target_step = target_step_y;
//		if(motor->target_step>=350)
//			motor->target_step=350;
//		else if(motor->target_step<=-350)
//			motor->target_step=-350;
//		motor->Anl_v = v;
//	}
//	if(No==2){
//		motor->pid_concroler->target_val = x_target;
//		pos_pid_realize(motor->pid_concroler,x_cur);
//		target_step_x += motor->pid_concroler->output;
//		motor->target_step = target_step_x;
//		if(motor->target_step>=350)
//			motor->target_step=350;
//		else if(motor->target_step<=-350)
//			motor->target_step=-350;
//		motor->Anl_v = v;
//	}
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
	pid_controller->epsilon_d1 = 20;
	pid_controller->epsilon_i1 = 20;
	pid_controller->epsilon_d2 = -20;
	pid_controller->epsilon_i2 = -20;
	pid_controller->output = 0;
	pid_controller->max = max;
	pid_controller->min = min;
	pid_controller->output_last = 0;
	pid_controller->i_max = 500;
}

/* 增量式PID实现函数 */
//void pos_pid_realize(pid* PID,PIDIn_Type actual_val)
//{
//	PID->cur_val = actual_val;
//	PID->err = PID->target_val - PID->cur_val;
//	//计算de(k)
//	PID->output = PID->kp*(PID->err - PID->err_k1) + PID->ki*PID->err + PID->kd*(PID->err + PID->err_k2 - 2*PID->err_k1);
//	PID->err_k2=PID->err_k1;
//	PID->err_k1=PID->err;
//}

/* 位置式PID实现函数 */
//mode：1：内环PID；2：外环PID
void pid_realize(pid *PID,PIDIn_Type actual_val,int mode)
{
	float switch_d = 1;
	int switch_i = 1;	//积分分离
	//int epsilon_d,epsilon_i;
	PID->cur_val = actual_val;
	PID->err = PID->target_val - PID->cur_val;
	if(mode==1){//内环
		PID->epsilon_d1 = 30;
		PID->epsilon_i1 = 30;
		PID->epsilon_d2 = -30;
		PID->epsilon_i2 = -30;
		PID->i_max = 3000;
	}
	else if(mode==2){//外环
		PID->i_max = 900;
	}
	//抗积分饱和
	if(PID->output_last>PID->max||PID->output_last<PID->min){
		if(PID->output_last*PID->err<0)//err使积分项绝对值减小
			PID->i += PID->err;
		else PID->i=PID->i;
	}
	else PID->i += PID->err;
	//积分分离和微分分离
	if(PID->err<PID->epsilon_d1&&PID->err>PID->epsilon_d2) switch_d = 0;
	if(PID->err>PID->epsilon_i1&&PID->err<PID->epsilon_i2) switch_i = 0;
	if(fabs_int(PID->err)<5) switch_i = 0;
	//积分限幅
	if(PID->i>PID->i_max) PID->i = PID->i_max;
	else if(PID->i<-PID->i_max) PID->i = -PID->i_max;
	PID->output = PID->kp*PID->err + switch_i*PID->ki*PID->i + switch_d*PID->kd*(PID->err - PID->err_k1);
	PID->err_k1 = PID->err;
	PID->output_last = PID->output;
	//输出限幅
	if(PID->output>PID->max) PID->output=PID->max;
	if(PID->output<PID->min) PID->output=PID->min;
}

//一阶滤波
float first_order_filter(float new_value,float last_value,float a)
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

