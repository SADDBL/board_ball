#include "connect.h"
#define angle_max 40

uint8_t RecieveBuffer[1];
uint8_t usart2_buf[9] = {0};
uint8_t RxLen2;
uint8_t flag2 = 0;

uint8_t RecieveBuffer3[1];
uint8_t usart3_buf[7] = {0};
uint8_t RxLen3;
uint8_t flag3 = 0;

float stepper_usart_angle[2]={0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	
	/***** USART3-树莓派串口 *****/
	if(huart->Instance == USART3)
	{

		if(flag3!=0){
			usart3_buf[RxLen3++] = *RecieveBuffer3;
			if(RxLen3==6){
				int i = 0,index = 100;
				RxLen3=0;
				flag3=0;
				x_cur = 0;
				y_cur = 0;
				for(i=0;i<3;i++){
					if(usart3_buf[i]==' ') x_cur+=0*(usart3_buf[i]-'0');
					else if(usart3_buf[i]>'/'&&usart3_buf[i]<':') x_cur+=index*(usart3_buf[i]-'0');
					index/=10;
				}
				index=100;
				for(i=0;i<3;i++){
					if(usart3_buf[i+3]==' ') y_cur+=0*(usart3_buf[i+3]-'0');
					else if(usart3_buf[i+3]>'/'&&usart3_buf[i+3]<':') y_cur+=index*(usart3_buf[i+3]-'0');
					index/=10;
				}
				//printf("%s\r\n",usart3_buf);
				//printf("x=%d,y=%d\r\n",x_cur,y_cur);
			}
		}
		else{
			if(*RecieveBuffer3=='a'){
				flag3=1;
				RxLen3=0;
			}
		}
		HAL_UART_Receive_IT(&huart3,RecieveBuffer3,1);
	}
	
	/***** USART2-调试串口 *****/
	else if(huart->Instance == USART2)
	{
		if(flag2!=0){
			usart2_buf[RxLen2++] = *RecieveBuffer;
			if(RxLen2==9||*RecieveBuffer=='z'){
				int angle=0,index=100,i=0;
				RxLen2=0;
				flag2=0;
				for(i=0;usart2_buf[i]!='z';i++){
					if(i==0){
						switch(usart2_buf[0])
						{
							case 'a':stepper_usart_angle[0] = 1;
											 break;
							case 'b':stepper_usart_angle[0] = 2;
											 break;
							case 'c':stepper_usart_angle[0] = 3;
											 break;
							case 'd':stepper_usart_angle[0] = 4;
											 break;
							default:printf("输入格式有误\r\n");
											break;
						}
						continue;
					}
					if(i==1&&usart2_buf[1]=='-'){
						index*=-1;
						continue;
					}
					if(usart2_buf[i]=='.'){
						i++;
						index/=10;
						continue;
					}
					angle += index*(usart2_buf[i]-'0');
					index/=10;
				}
				if(angle>=angle_max) angle = angle_max;
				else if(angle<=-angle_max) angle = -angle_max;
				stepper_usart_angle[1] = angle;
				
				printf("%0.f:\t",stepper_usart_angle[0]);
				printf("%f\r\n",stepper_usart_angle[1]);
			}
		}
		else{
			if(*RecieveBuffer=='a'||*RecieveBuffer=='b'||*RecieveBuffer=='c'||*RecieveBuffer=='d'){
				flag2 = 1;
				RxLen2 = 0;
				usart2_buf[RxLen2++] = *RecieveBuffer;
			}
		}
		HAL_UART_Receive_IT(&huart2, (uint8_t *)RecieveBuffer, 1);
	}
}

