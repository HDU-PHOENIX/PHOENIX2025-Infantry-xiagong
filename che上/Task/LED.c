#include "LED.h"
#include "BSP_CAN.h"
#include "gimbal_task.h"

uint8_t Data_clear[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
void led_task(void const * argument)
{
	
 while(1)
 {
 HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
 HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
 HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
	if(Gimbal_Motor_Yaw.motor_value->Error == 0 ||Gimbal_Motor_Pitch.motor_value->Error == 0){
		Motor_enable();
		
	}else if(Gimbal_Motor_Yaw.motor_value->Error != 1 ||Gimbal_Motor_Pitch.motor_value->Error != 1)
            {
            	CANx_SendStdData(&hcan2,0x109,Data_clear,8);
							CANx_SendStdData(&hcan2,0x107,Data_clear,8);
            }

 osDelay(1000);

	
	 
 }
}


