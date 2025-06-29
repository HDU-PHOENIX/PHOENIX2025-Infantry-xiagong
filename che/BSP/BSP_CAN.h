#ifndef __BSP_CAN
#define __BSP_CAN


//#include "stm32f4xx_hal.h"
#include "can.h"
#include "main.h"
#include "type.h"
/*CAN发送或是接收的ID*/

#define set_chassis_current  set_moto1234_current
#define set_gimbal_current set_moto5678_current
#define set_magazine_current set_moto5678_current
#define set_friction1_voltage __HAL_TIM_SetCompare
#define set_friction2_voltage __HAL_TIM_SetCompare

/*接收到的云台电机的参数结构体*/



/* Extern  ------------------------------------------------------------------*/
extern moto_measure_t  moto_CAN1[11];
extern moto_measure_t moto_CAN2[11];
extern C_Board_measure_t C_Board;
extern float cap_remain_percent;

void my_can_filter_init(CAN_HandleTypeDef* hcan);
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id);
void get_moto_measure(moto_measure_t *ptr,uint8_t temp[]);
void get_4310moto_measure(uint8_t temp[],moto_measure_t *ptr);
void get_C_Board_measure(C_Board_measure_t *ptr,uint8_t temp[]);
void can_receive_onetime(CAN_HandleTypeDef* _hcan, int time);
void set_moto1234_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
void set_moto5678_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
void set_moto91011_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3);
void get_total_angle(moto_measure_t *p);

void Motor_enable(void);
uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq);

#define Motar_mode 0	//设置模式为何种模式，为0为IMT模式，为1为位置速度模式，为2为速度模式

#define P_MIN -3.141593		//位置最小值
#define P_MAX 3.141593		//位置最大值
#define V_MIN -30			//速度最小值
#define V_MAX 30			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -10			//转矩最大值
#define T_MAX 10			//转矩最小值

#define X_MIN 	-3.000		//X速度最小值
#define X_MAX 	 3.000		//X速度最大值
#define Y_MIN 	-3.000		//Y速度最小值
#define Y_MAX 	 3.000		//Y速度最大值
#define W_MIN 	-9.000		//Z速度最小值
#define W_MAX 	 9.000		//Z速度最大值
#endif
