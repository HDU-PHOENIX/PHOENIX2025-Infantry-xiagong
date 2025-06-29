#ifndef GIMBALTASKH
#define GIMBALTASKH

#include "main.h"
#include "type.h"


/* 云台操作模式:
   
   普通             	NORMAL
   调头180°             AROUND
   打符             	BUFF
   补弹,pitch水平   	LEVEL
   机械模式pitch抬头	HIGH
   快速扭头90°          TURN
*/
typedef enum
{
	GIMBAL_NORMAL  					= 0,			//正常模式,进行模式选择
	GIMBAL_TURN_RIGHT  			= 1,			//右转90°调头
	GIMBAL_CHASSIS_FOLLOW   = 2,			//
	GIMBAL_LEVEL   					= 3,			//弹仓开启,云台水平
	GIMBAL_MANUAL  					= 4,			//手动打符模式
	GIMBAL_SM_BUFF 					= 5,			//小符
	GIMBAL_TURN_LEFT    		= 7,			//左转90°扭头
	GIMBAL_AUTO    					= 8,			//自瞄
	GIMBAL_BASE    					= 9,			//桥头吊射基地
	GIMBAL_BUFF 						=	10,			//打小符
	GIMBAL_BUFFBUFF 				= 11,	
	GIMBAL_PREDICT   				=	12,  		//预测模式中的自瞄
	GIMBAL_GYROSCOPE 				=	13,			//小陀螺
}eGimbalAction;
extern eGimbalAction  actGimbal;

typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}speed_calc_data_t;


typedef struct
{
    float Yaw_P;
    float Yaw_V;
    float Pitch_P;
		float Pitch_V;
} Gimbal_Speed;


typedef enum
{
	USEENCODER,
	USEIMU
}GimbalModeType;
extern GimbalModeType YawGimbalMode ;
extern GimbalModeType PitchGimbalMode ;

extern Motortype 			Gimbal_Motor_Yaw;
extern Motortype 			Gimbal_Motor_Pitch;

extern float trya;

void GimbalFun(void const * argument);
void GIMBAL_InitArgument(void);
void RemoteControlGimbal(void);
void Gimbal_Single_Loop_Out(void);

#endif

