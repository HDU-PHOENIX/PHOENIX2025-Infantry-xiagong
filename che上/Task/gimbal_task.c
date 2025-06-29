#include "gimbal_task.h"
#include "IMU_Task.h"
#include "type.h"
#include "param.h"
#include "math.h"
#include "arm_math.h"
#include "dbus.h"
#include "BSP_CAN.h"
#include "cmsis_os.h"
#include "tim.h"
#include "pid.h"
#include "vision.h"
#include "chassis_task.h"


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
eGimbalAction  	actGimbal_L				=		GIMBAL_NORMAL;
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
fp32 Gimbal_Yaw_Encoder_Position_pid				[3]	=	{0.7,			0,			7.6};//{0.9,			0,			10.6};//{12,			0.004,			0.83};i
fp32 Gimbal_Yawspeed_pid										[3]	=	{0.61,			0.020,			0.73};
fp32 Gimbal_Pitch_Encoder_Position_pid			[3]	=	{0.31,			0,			1.24};//{0.55,			0,			2.1};
fp32 Gimbal_Pitch_Speed_pid_Start						[3]	=	{0.52,			0.03,			0.07};//{0.96,			0.07,			0.15};

Gimbal_Speed 		absolute_gimbal_speed;

void GimbalFun(void const * argument){

	portTickType currentTime;
	GIMBAL_InitArgument();
	osDelay(2000);
	//Motor_enable();	
	while(1)
	{
		currentTime = xTaskGetTickCount();			//当前系统时间
		switch(ControlMode)
			{
				case KEYBOARD:
				{
						GIMBAL_Mode_Choose();
						GIMBAL_Key_Ctrl(); 
						Gimbal_Single_Loop_Out();
					break;
				}
				case REMOTE:
				{
						if(rc.sw1 == 2 ){
                actGimbal=GIMBAL_CHASSIS_FOLLOW;
						}	//底盘跟随云台
						else if(rc.sw1 == 1 && rc.sw2==2){
                actGimbal=GIMBAL_SLOW;
								//ControlMode	=	KEYBOARD;
						}	//底盘不跟随云台
            else if(rc.sw1 == 3 && rc.sw2==2){
                actGimbal=GIMBAL_SLOW	;
						}		//静默模式   //CHASSIS_SLOW  CHASSIS_GYROSCOPE
            else if(rc.sw1 == 3 ){
                actGimbal=GIMBAL_GYROSCOPE;
						}		//小陀螺模式 
						else{
								actGimbal=GIMBAL_SLOW;
						}
            RemoteControlGimbal();								//解算遥控器速度到云台速度
           Gimbal_Single_Loop_Out();						//整车运动速度解算出云台运动速度
					break;
				}
				case FUWEI:
				{
					break;
				}     
				default:
					break;
			}
//			number+=1;
		vision_send();
//			coshf(1);
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
	
	pid_param_init(&Gimbal_Motor_Yaw.Motor_PID_Position,		PID_POSITION,		Gimbal_Yaw_Encoder_Position_pid,		15.0,		3.0,		30.0,		0,		0,		0,			0);//i:15
	Gimbal_Motor_Yaw.Motor_PID_Position.angle_max 		= 		180;
	Gimbal_Motor_Yaw.Motor_PID_Position.angle_min 		= 		-180;
	pid_param_init(&Gimbal_Motor_Yaw.Motor_PID_Speed,				PID_POSITION,		Gimbal_Yawspeed_pid,								6.0,		2.0,		4,		0.0,		0,	0,			0);
    
	Gimbal_Motor_Pitch.ID						=		0x109;
	Gimbal_Motor_Pitch.motor_value	=		&moto_CAN2[10];
	
	pid_param_init(&Gimbal_Motor_Pitch.Motor_PID_Position,	PID_POSITION,		Gimbal_Pitch_Encoder_Position_pid,	3,	1,		15,		0,		0,		0,		0);
	Gimbal_Motor_Pitch.Motor_PID_Position.angle_max 		= 		180;
	Gimbal_Motor_Pitch.Motor_PID_Position.angle_min 		= 		-180;
	pid_param_init(&Gimbal_Motor_Pitch.Motor_PID_Speed,			PID_POSITION,		Gimbal_Pitch_Speed_pid_Start,				6,	2,	1.5,		0,		0,						0,			0);
	absolute_gimbal_speed.Yaw_P	=	IMU_Chass_MSG.angle[0];
	absolute_gimbal_speed.Pitch_P =0;
}


/**
  * @brief  遥控器控制方式
  * @param  void
  * @retval void
  * @attention
  *主要处理absolute_chassis_speed，底盘的运动方向，来进行逆运动学处理
  */

int A1=0;
void	RemoteControlGimbal(){
	switch(actGimbal) {
    case GIMBAL_CHASSIS_FOLLOW://跟随云台
			
				Gimbal_Control_Yaw();
				Gimbal_Control_Pitch();
        break;
    case GIMBAL_NORMAL://不跟随云台
			
				Gimbal_Control_Yaw();
				Gimbal_Control_Pitch();

        break;
				
    case GIMBAL_GYROSCOPE:		//小陀螺模式
			
				Gimbal_Control_Yaw();
				Gimbal_Control_Pitch();

        break;
		case GIMBAL_AUTO:		
				if(actGimbal	!=	actGimbal_L){
					vision_clear_R();
				}
				Gimbal_Control_Yaw_Auto();
				Gimbal_Control_Pitch_Auto();
				Gimbal_Vision();
				break;
		case GIMBAL_SLOW://不跟随云台

        break;	
			 
    default:
        break;
    }
	actGimbal_L=actGimbal;
}



float trya=0;
float jiao=0;
float pitch=0;
void Gimbal_Single_Loop_Out(){


	Gimbal_Motor_Yaw.motor_value->target_angle=absolute_gimbal_speed.Yaw_P;
	Gimbal_Motor_Pitch.motor_value->target_angle=absolute_gimbal_speed.Pitch_P;
	
	pid_caculate(&Gimbal_Motor_Yaw.Motor_PID_Position,			IMU_Chass_MSG.angle[0],		Gimbal_Motor_Yaw.motor_value->target_angle);
  pid_caculate(&Gimbal_Motor_Yaw.Motor_PID_Speed,					Gimbal_Motor_Yaw.motor_value->velocity,		-Gimbal_Motor_Yaw.Motor_PID_Position.out);
	
	if(Gimbal_Motor_Yaw.motor_value->Error == 1 &&	Gimbal_Motor_Pitch.motor_value->Error == 1){
		
	pid_caculate(&Gimbal_Motor_Pitch.Motor_PID_Position,			IMU_Chass_MSG.angle[1],		absolute_gimbal_speed.Pitch_P*180/3.141593);
  pid_caculate(&Gimbal_Motor_Pitch.Motor_PID_Speed,					Gimbal_Motor_Pitch.motor_value->velocity,-Gimbal_Motor_Pitch.Motor_PID_Position.out	);
	pitch	=	Gimbal_Motor_Pitch.Motor_PID_Speed.out	+	0.820*cosf(IMU_Chass_MSG.angle[1]	/	180.0	*3.141593);
	}
		
	
		//pitch=0;
	  MIT_CtrlMotor(&hcan2,Gimbal_Motor_Yaw.ID,0, 0,0,0,Gimbal_Motor_Yaw.Motor_PID_Speed.out);
		for(int i=0;i<6000;i++);
		MIT_CtrlMotor(&hcan2,Gimbal_Motor_Pitch.ID,0, 0,0,0,pitch);//Gimbal_Motor_Pitch.Motor_PID_Speed.out);
}


void Gimbal_Control_Yaw(void){
				absolute_gimbal_speed.Yaw_P = absolute_gimbal_speed.Yaw_P -(float)rc.ch3/1000;			
				if(absolute_gimbal_speed.Yaw_P	<=	-180.0){
						absolute_gimbal_speed.Yaw_P	= 180.0*2+absolute_gimbal_speed.Yaw_P;
				}else	if(absolute_gimbal_speed.Yaw_P	>	180.0){
						absolute_gimbal_speed.Yaw_P	= -180.0*2+absolute_gimbal_speed.Yaw_P;
				}
}
void Gimbal_Control_Pitch(void){
			if(Gimbal_Motor_Pitch.motor_value->Error == 1){
//				if(Gimbal_Motor_Pitch.motor_value->position >	0.70){
//					absolute_gimbal_speed.Pitch_P = absolute_gimbal_speed.Pitch_P + 0.0004;
//					A1=-1;
//				}else if(Gimbal_Motor_Pitch.motor_value->position <	-0.35){
//					absolute_gimbal_speed.Pitch_P = absolute_gimbal_speed.Pitch_P - 0.0004;
//					A1=1;
//				}else if(rc.ch4*A1>=0){
					absolute_gimbal_speed.Pitch_P	=	absolute_gimbal_speed.Pitch_P -	(float)rc.ch4/80000;
//					A1=0;
//				}
		}
		if(absolute_gimbal_speed.Pitch_P>3.141593/2	||absolute_gimbal_speed.Pitch_P<-3.141593/3){
			absolute_gimbal_speed.Pitch_P=0;
		}

}

void GIMBAL_Mode_Choose(void){

	if(IF_MOUSE_PRESSED_RIGH	&&	Vision_Read_Data.find_bool=='1'){
		actGimbal = GIMBAL_AUTO;//自瞄
	}else{
		actGimbal = GIMBAL_NORMAL;
	}
  
	actGimbal_L=actGimbal;
}

void	GIMBAL_Key_Ctrl(){
	switch(actGimbal) {
    case GIMBAL_CHASSIS_FOLLOW://跟随云台
			
				Gimbal_KeyControl_Yaw();
				Gimbal_KeyControl_Pitch();
        break;
    case GIMBAL_NORMAL://不跟随云台
			
				Gimbal_KeyControl_Yaw();
				Gimbal_KeyControl_Pitch();

        break;
				
    case GIMBAL_GYROSCOPE:		//小陀螺模式
			
				Gimbal_KeyControl_Yaw();
				Gimbal_KeyControl_Pitch();

        break;
		case GIMBAL_AUTO:		
				if(actGimbal	!=	actGimbal_L){
					vision_clear_R();
				}
				Gimbal_Control_Yaw_Auto();
				Gimbal_Control_Pitch_Auto();
				Gimbal_Vision();
				break;
		case GIMBAL_SLOW://不跟随云台

        break;	
			 
    default:
        break;
    }
	actGimbal_L=actGimbal;
}

void Gimbal_KeyControl_Yaw(void){
				absolute_gimbal_speed.Yaw_P = absolute_gimbal_speed.Yaw_P -	(float)MOUSE_X_MOVE_SPEED/100.0;			
				if(absolute_gimbal_speed.Yaw_P	<=	-180.0){
						absolute_gimbal_speed.Yaw_P	= 180.0*2+absolute_gimbal_speed.Yaw_P;
				}else	if(absolute_gimbal_speed.Yaw_P	>	180.0){
						absolute_gimbal_speed.Yaw_P	= -180.0*2+absolute_gimbal_speed.Yaw_P;
				}
}
void Gimbal_KeyControl_Pitch(void){
		if(Gimbal_Motor_Pitch.motor_value->Error == 1){
//				if(Gimbal_Motor_Pitch.motor_value->position >	0.70){
//					absolute_gimbal_speed.Pitch_P = absolute_gimbal_speed.Pitch_P + 0.0004;
//					A1=-1;
//				}else if(Gimbal_Motor_Pitch.motor_value->position <	-0.35){
//					absolute_gimbal_speed.Pitch_P = absolute_gimbal_speed.Pitch_P - 0.0004;
//					A1=1;
//				}else if(rc.ch4*A1>=0){
					absolute_gimbal_speed.Pitch_P	=	absolute_gimbal_speed.Pitch_P +	(float)MOUSE_Y_MOVE_SPEED/8000.0;
//					A1=0;
//				}
		}
		if(absolute_gimbal_speed.Pitch_P>3.141593/2	||absolute_gimbal_speed.Pitch_P<-3.141593/3){
			absolute_gimbal_speed.Pitch_P=0;
		}
}





void Gimbal_Control_Yaw_Auto(void){
				//absolute_gimbal_speed.Yaw_P = absolute_gimbal_speed.Yaw_P +Vision_Read_Data.yaw;
				absolute_gimbal_speed.Yaw_P=Vision_Read_Data.yaw/3.141593*180.0;
				if(absolute_gimbal_speed.Yaw_P	<=	-180.0){
						absolute_gimbal_speed.Yaw_P	= 180.0*2+absolute_gimbal_speed.Yaw_P;
				}else	if(absolute_gimbal_speed.Yaw_P	>	180.0){
						absolute_gimbal_speed.Yaw_P	= -180.0*2+absolute_gimbal_speed.Yaw_P;
				}
}
void Gimbal_Control_Pitch_Auto(void){
				if(Gimbal_Motor_Pitch.motor_value->position >	0.70){
					absolute_gimbal_speed.Pitch_P = absolute_gimbal_speed.Pitch_P + 0.0004;
					A1=-1;
				}else if(Gimbal_Motor_Pitch.motor_value->position <	-0.35){
					absolute_gimbal_speed.Pitch_P = absolute_gimbal_speed.Pitch_P - 0.0004;
					A1=1;
				}else if((absolute_gimbal_speed.Pitch_P	-Vision_Read_Data.pitch)*A1>=0){
					//absolute_gimbal_speed.Pitch_P	=	absolute_gimbal_speed.Pitch_P +	Vision_Read_Data.pitch;
					absolute_gimbal_speed.Pitch_P =	Vision_Read_Data.pitch;
					A1=0;
				}
		if(absolute_gimbal_speed.Pitch_P>3.141593/2	||absolute_gimbal_speed.Pitch_P<-3.141593/3){
			absolute_gimbal_speed.Pitch_P=0;
		}
}
void Gimbal_Vision(void){
//	Vision_Send_Data.type  						= 		0xB0;
//	Vision_Send_Data.yaw 							=			IMU_Chass_MSG.angle[0];
//	Vision_Send_Data.pitch 						=			IMU_Chass_MSG.angle[1];
//	Vision_Send_Data.enemy_team_color	= 		0;
//	Vision_Send_Data.mode							=			0;
//	Vision_Send_Data.rune_flag				=			0;

}

