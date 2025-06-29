#include "gimbal_task.h"
#include "type.h"
#include "param.h"
#include "math.h"
#include "arm_math.h"
#include "dbus.h"
#include "BSP_CAN.h"
#include "cmsis_os.h"
#include "tim.h"
#include "pid.h"

Motortype 			Gimbal_Motor_Yaw;
Motortype 			Gimbal_Motor_Pitch;

//双轴PID
PidTypeDef			Gimbal_Yawimu_PID;					//自瞄yaw
PidTypeDef 			Gimbal_Pitchimu_PID;				//自瞄pitch和操作pitch

PidTypeDef 			Gimbal_YawimuBUFF_PID;			//打符pid
PidTypeDef 			Gimbal_PitchimuBUFF_PID;

PidTypeDef 			Gimbal_Yaw_PID;							//操作yaw
//云台模式选择\\Robot\../Task/gimbal_task.c\Gimbal_Pitchimu_PID.Kp
eGimbalAction  	actGimbal					=		GIMBAL_NORMAL;
GimbalModeType 	YawGimbalMode 		=		USEIMU;
GimbalModeType 	PitchGimbalMode 	=		USEIMU;
//自瞄用PID
PidTypeDef 			VISION_PITCH;
PidTypeDef 			VISION_YAW;			


fp32 Gimbal_Yawimu_pid											[3]	=	{0,			0,			0};
fp32 Gimbal_Pitchimu_pid										[3]	=	{0,			0,			0};
fp32 pid_yaw																[3]	=	{0,			0,			0};
fp32 yaw_buff_pid														[3]	=	{0,			0,			0};
fp32 pitch_buff_pid													[3]	=	{0,			0,			0};
fp32 Gimbal_Yaw_Encoder_Position_pid				[3]	=	{8,			0.01,			0.05};
fp32 Gimbal_Yawspeed_pid										[3]	=	{0.55,			0.06,			0.5};
fp32 Gimbal_Pitch_Encoder_Position_pid			[3]	=	{0,			0,			0};
fp32 Gimbal_Pitch_Speed_pid_Start						[3]	=	{0,			0,			0};

Gimbal_Speed 		absolute_gimbal_speed;


void GimbalFun(void const * argument){

	portTickType currentTime;
	GIMBAL_InitArgument();
//	osDelay(1500);
//	Motor_enable();	
	while(1)
	{
		currentTime = xTaskGetTickCount();			//当前系统时间
		switch(ControlMode)
			{
				case KEYBOARD:
				{
//						GIMBAL_Mode_Choose();
//						GIMBAL_Key_Ctrl(); 
//						GIMBAL_Double_Loop_Out();
					break;
				}
				case REMOTE:
				{
						if(rc.sw1 == 2 && rc.sw1==2){
                actGimbal=GIMBAL_CHASSIS_FOLLOW;
						}	//底盘跟随云台
					else if(rc.sw1 == 2 && rc.sw2==1){
                actGimbal=GIMBAL_NORMAL;
						}	//底盘不跟随云台
//            else if(rc.sw1 == 3 && rc.sw2==2){
//                actGimbal=CHASSIS_SLOW;
//						}		//射击模式   //CHASSIS_SLOW  CHASSIS_GYROSCOPE
            else if(rc.sw1 == 3 && rc.sw2==1){
                actGimbal=GIMBAL_GYROSCOPE;
						}		//小陀螺模式                                                       
//            RemoteControlGimbal();								//解算遥控器速度到云台速度
           // Gimbal_Single_Loop_Out();						//整车运动速度解算出云台运动速度
					break;
				}
				case FUWEI:
				{
					break;
				}     
				default:
					break;
			}
		vTaskDelayUntil(&currentTime, 2);//绝对延时
	}
}

/**
  * @brief  云台参数初始化
  * @param  void
  * @retval void
  * @attention 没有加I会有误差,只在系统启动时调用一次
  */

void GIMBAL_InitArgument(void){

  pid_param_init(&Gimbal_Yawimu_PID,				PID_POSITION,		Gimbal_Yawimu_pid,      800,      400,       	200,             0,         0,      360,           0);
	pid_param_init(&Gimbal_Pitchimu_PID,			PID_POSITION,		Gimbal_Pitchimu_pid,		200,      5,       		200,             0,         0,      360,           0);
	pid_param_init(&Gimbal_Yaw_PID,						PID_POSITION,		pid_yaw,								200,      5,       		200,             0,         0,      360,           0);
  pid_param_init(&Gimbal_YawimuBUFF_PID,		PID_POSITION,		yaw_buff_pid,						800,      400,       	200,             0,         0,      360,           0);
	pid_param_init(&Gimbal_PitchimuBUFF_PID,	PID_POSITION,		pitch_buff_pid,					200,      5,       		200,             0,         0,      360,           0);
  //Yaw Pitch 电机
	//电机初始化
	Gimbal_Motor_Yaw.ID							=		0x107;
	Gimbal_Motor_Yaw.motor_value		=		&moto_CAN2[9];
	
	pid_param_init(&Gimbal_Motor_Yaw.Motor_PID_Position,		PID_POSITION,		Gimbal_Yaw_Encoder_Position_pid,		30,		30,		3.141593/6,		0,		0,		0,			0);
	Gimbal_Motor_Yaw.Motor_PID_Position.angle_max 		= 		3.141593;
	Gimbal_Motor_Yaw.Motor_PID_Position.angle_min 		= 		-3.141593;
	pid_param_init(&Gimbal_Motor_Yaw.Motor_PID_Speed,				PID_POSITION,		Gimbal_Yawspeed_pid,								10,	10,	3,		0,		0,	0,			0);
    
	Gimbal_Motor_Pitch.ID						=		0x109;
	Gimbal_Motor_Pitch.motor_value	=		&moto_CAN2[10];
	
	pid_param_init(&Gimbal_Motor_Pitch.Motor_PID_Position,	PID_POSITION,		Gimbal_Pitch_Encoder_Position_pid,	400,		200,		3e38,		0,		0.899999976,		8192,		0);
	pid_param_init(&Gimbal_Motor_Pitch.Motor_PID_Speed,			PID_POSITION,		Gimbal_Pitch_Speed_pid_Start,				30000,	10000,	3e38,		0,		0.1,						0,			0);
	absolute_gimbal_speed.Yaw_P	=	0;
}


/**
  * @brief  遥控器控制方式
  * @param  void
  * @retval void
  * @attention
  *主要处理absolute_chassis_speed，底盘的运动方向，来进行逆运动学处理
  */

void	RemoteControlGimbal(){
	switch(actGimbal) {
    case GIMBAL_CHASSIS_FOLLOW://跟随云台
			
				absolute_gimbal_speed.Yaw_P = absolute_gimbal_speed.Yaw_P +(float)rc.ch3/80000;
				if(absolute_gimbal_speed.Yaw_P	<	-3.141593){
						absolute_gimbal_speed.Yaw_P	= 3.141593*2+absolute_gimbal_speed.Yaw_P;
				}else	if(absolute_gimbal_speed.Yaw_P	>	3.141593){
						absolute_gimbal_speed.Yaw_P	= -3.141593*2+absolute_gimbal_speed.Yaw_P;
				}
        break;                           
    case GIMBAL_NORMAL://不跟随云台
			
				absolute_gimbal_speed.Yaw_P = absolute_gimbal_speed.Yaw_P +(float)rc.ch3/80000;
				if(absolute_gimbal_speed.Yaw_P	<	-3.141593){
						absolute_gimbal_speed.Yaw_P	= 3.141593*2+absolute_gimbal_speed.Yaw_P;
				}else	if(absolute_gimbal_speed.Yaw_P	>	3.141593){
						absolute_gimbal_speed.Yaw_P	= -3.141593*2+absolute_gimbal_speed.Yaw_P;
				}
        break;
    case GIMBAL_GYROSCOPE:		//小陀螺模式

        break;
//	case CHASSIS_SLOW:		//静态模式

//        break;
    default:
        break;
    }
}



float trya=0;
float jiao=0;
void Gimbal_Single_Loop_Out(){
	jiao=(GIMBAL_YAW_X	-	Gimbal_Motor_Yaw.motor_value->position);
		if(jiao	<	-3.141593){
				jiao	= 3.141593+jiao+3.141593;
		}else	if(jiao	>	3.141593){
				jiao	= -3.141593*2+jiao;
		}

	Gimbal_Motor_Yaw.motor_value->target_angle=absolute_gimbal_speed.Yaw_P;
	
	//pid_caculate(&Gimbal_Motor_Yaw.Motor_PID_Position,			jiao,		Gimbal_Motor_Yaw.motor_value->target_angle);
  pid_caculate(&Gimbal_Motor_Yaw.Motor_PID_Speed,					Gimbal_Motor_Yaw.motor_value->velocity,	trya);	//-Gimbal_Motor_Yaw.Motor_PID_Position.out);

	MIT_CtrlMotor(&hcan2,Gimbal_Motor_Yaw.ID,0, 0,0,0, Gimbal_Motor_Yaw.Motor_PID_Speed.out);
}

