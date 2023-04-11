#include "Hardware.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/***** OLED��Ļ��ʾ���������� *****/

/* ��ʾ���� */
/**
 * @brief  ��ʾpid����
 * @param  pid_No: pid��������ţ�1��stepper1��2��stepper2
 */
void oled_pid_para_dis(int pid_No)
{
	char *p_str;
	char *i_str;
	char *d_str;
	p_str=(char*)malloc(sizeof(char)*8);
	i_str=(char*)malloc(sizeof(char)*8);
	d_str=(char*)malloc(sizeof(char)*8);
	memset(p_str,'\0',8);
	memset(i_str,'\0',8);
	memset(d_str,'\0',8);
	if(pid_No==1)	//��ʾstepper1��pid����
	{
		OLED_ShowString(1,0,"M1",16);
		sprintf(p_str,"p:=%f",kp1);
		sprintf(i_str,"i:=%f",ki1);
		sprintf(d_str,"d:=%f",kd1);
		OLED_ShowString(1,15,p_str,16);
		OLED_ShowString(1,31,i_str,16);
		OLED_ShowString(1,47,d_str,16);
		OLED_Refresh();
	}
	else if(pid_No==2)	//��ʾstepper2��pid����
	{
		OLED_ShowString(1,0,"M2",16);
		sprintf(p_str,"p:=%f",kp2);
		sprintf(i_str,"i:=%f",ki2);
		sprintf(d_str,"d:=%f",kd2);
		OLED_ShowString(1,15,p_str,16);
		OLED_ShowString(1,31,i_str,16);
		OLED_ShowString(1,47,d_str,16);
		OLED_Refresh();
	}
}
