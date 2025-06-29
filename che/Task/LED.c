#include "LED.h"
#include "tuxin.h"
#include "main.h"
#include "param.h"
#include "type.h"
#include "usart.h"
#include "gimbal_task.h"

uint8_t str[17] = {"cap:"};
uint8_t shoot_str[17]={"shoot:"};
uint8_t chassis_gyro1[4][200]={{0},{0},{0},{0}};  // 创建指向二维数组的指针并将其指向数组的第一行

void led_task(void const * argument)
{
	
//	portTickType currentTime;
//	uint16_t UI_PushUp_Counter = 0;
	while(1)
	{
//		UI_PushUp_Counter++;
//		currentTime = xTaskGetTickCount();
//		show_str(shoot_str,17,2,100,700,add,1,huart6);
//		show_str(str,17,3,100,650,add,2,huart6);
//		draw_five_line(chassis_gyro1,0,huart6,pose);
//		yaw_error_cal();
//		draw_five_line(chassis_gyro1,0,huart6,pose1);
		
		
//	if(UI_PushUp_Counter%5==0){
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
//	}
//if(UI_PushUp_Counter>500){UI_PushUp_Counter=0;}
	//vTaskDelayUntil(&currentTime, 1000);//绝对延时
 osDelay(500);
	 
 }
}

void yaw_error_cal()
{		
	float substarct_angle;	//云台和底盘编码器差值的角度 
	substarct_angle=(Gimbal_Motor_Yaw.motor_value->position-GIMBAL_YAW_X)/3.141596*180; //不知道灯条是不是这个位置
	float start_angle,end_angle;
	start_angle = 30 + substarct_angle;
	if(start_angle < 0)
		start_angle += 360;
	end_angle = 330 + substarct_angle;
	if(end_angle > 360)
		end_angle -= 360;
	pose1[1][4] = (uint32_t)start_angle;
	pose1[1][5] = (uint32_t)end_angle;
	pose1[2][4] = (uint32_t)end_angle;
	pose1[2][5] = (uint32_t)start_angle;
}	
	
