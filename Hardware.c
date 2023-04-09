#include "Hardware.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int MENU = 0;
char str1[10] = "/0",str2[10] = "/0",str3[10] = "/0";

/***** OLED��Ļ��ʾ *****/

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

/* ����main���� */
/**
 * @brief  ��ʾmain����
 */
void oled_main_menu_draw(void){
	OLED_ShowString(32,0,"Main Menu",16);
	OLED_ShowString(1,1*16,"Init:1",16);
	OLED_ShowString(8*8,1*16,"Task:2",16);
	OLED_Refresh();
}

/* main����������� */
void oled_main_menu_opera(void){
	int static main_flag=0;
	if(MENU!=MAIN_MENU){
	}
	//����main����
	if(main_flag==0){
		main_flag=1;
		oled_main_menu_draw();
		keyborad_data = 0;
	}
	//main�������
	//ѡ��set����
	if(keyborad_data==1){
		keyborad_data = 0;
		MENU = SET_MENU;
		return;
	}
	//task����
	else if(keyborad_data==2){
		keyborad_data = 0;
		MENU = TASK_MENU;
		return;
	}
}

/* set����������� */
/**
 * @brief  set�����������
 */
void oled_set_menu_opera(void){
	static int keyboard_data_last;
	int i;
	OLED_ShowString(32,0,"Set Menu",16);
	OLED_Refresh();
	//�ж��Ƿ���Set����
	if(MENU!=SET_MENU){}
	if(keyborad_data<6&&keyborad_data>0){
	//if(keyborad_data!=keyboard_data_last&&(keyborad_data<6&&keyborad_data>0)){
		//2->3
		for(i=0;str2[i]!='\0';i++) str3[i] = str2[i];
		for(;i<10;i++) str3[i] = '\0';
		//1->2
		for(i=0;str1[i]!='\0';i++) str2[i] = str1[i];
		for(;i<10;i++) str2[i] = '\0';
		//���õ������
		point_list[keyborad_data-1].x = x_cur;
		point_list[keyborad_data-1].y = y_cur;
		sprintf(str1,"%d:%d,%d",keyborad_data,point_list[keyborad_data-1].x,point_list[keyborad_data-1].y);
		//��ʾ������
		OLED_ShowString(1,1*16,str1,16);
		OLED_ShowString(1,2*16,str2,16);
		OLED_ShowString(1,3*16,str3,16);
		keyborad_data = 0;
	}
	//10�Ű���������main����
	else if(keyborad_data == 10){
		MENU = MAIN_MENU;
		keyborad_data = 0;
		return;
	}
}

/* ����task���� */
void oled_task_menu_draw(void){
	OLED_ShowString(32,0,"Task Menu",16);
	//��������
	OLED_ShowString(1,1*16,"A:1,2,3,4",16);
	//���ײ���
	OLED_ShowString(1,2*16,"B:5,6,7",16);
}

/* task�������� */
void oled_task_menu_opera(void){
	
}
