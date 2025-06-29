#include "shoot_task.h"
#include "type.h"
#include "param.h"
#include "math.h"
#include "arm_math.h"
#include "dbus.h"
#include "BSP_CAN.h"
#include "cmsis_os.h"
#include "tim.h"
#include "pid.h"


eShootAction  	actShoot					=		SHOOT_NORMAL;
eShootAction  	actShoot_L					;
Motortype 			Shoot_Motor[2];

fp32 Shoot_Pid[3]	=	{15,			0,			24.9};

uint8_t		bodan_speed			=	0;
int 		shoot_speed 		= 0;

uint8_t b_b,b_r,b_left;



void ShootFun(void const * argument){
	
	portTickType currentTime;
	Shoot_InitArgument();
	osDelay(300);
	while(1)
	{
		currentTime = xTaskGetTickCount();			//当前系统时间
		switch(ControlMode)
			{
				case KEYBOARD:
				{
						Shoot_Mode_Choose();
						Shoot_Key_Ctrl(); 
						Shoot_Single_Loop_Out();	
					break;
				}
				case REMOTE:
				{
						if((rc.sw1 == 1 && rc.sw2==2)||(rc.sw1 == 3 && rc.sw2==1)||(rc.sw1 == 2 && rc.sw2==1)){
                actShoot	=	SHOOT_NORMAL;
						}	//正常模式
						else if(rc.sw1 == 1 && rc.sw2==3){
                actShoot	=	SHOOT_SINGLE;
						}	//单发模式
						else{
								actShoot	=	SHOOT_SLOW;
						}	//静默模式
                                                       
            RemoteControlShoot();								
            Shoot_Single_Loop_Out();						
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







void Shoot_InitArgument(void){

		Shoot_Motor[0].ID	=	0x201;
		Shoot_Motor[0].motor_value=&moto_CAN1[0];
		Shoot_Motor[1].ID	=	0x202;
		Shoot_Motor[1].motor_value=&moto_CAN1[1];
	
		pid_param_init(&(Shoot_Motor[0].Motor_PID_Speed),      PID_POSITION,  Shoot_Pid,16384,   16384,     400,          0,       0.1,    0,            0 ); //16384
		pid_param_init(&(Shoot_Motor[1].Motor_PID_Speed),      PID_POSITION,  Shoot_Pid,16384,   16384,     400,          0,       0.1,    0,            0 ); //16384
	
}






void RemoteControlShoot(void){
		switch(actShoot) {
    case SHOOT_NORMAL://正常模式
			C_Board.mode	=	1;
				shoot_speed = 6600;
		if(rc.wheel>0){
				bodan_speed	= 1;
		}else if(rc.wheel==0){
				bodan_speed	= 0;
		}else{
				bodan_speed	= 254;
		}
        break;
    case SHOOT_SINGLE://单发模式
			C_Board.mode	=	2;

        break;
				
    case SHOOT_SLOW:	//静默模式
			C_Board.mode	=	0;
			shoot_speed = 0;
			bodan_speed = 0;

        break;
				
    default:
        break;
    }


}





void Shoot_Single_Loop_Out(void){

	Shoot_Motor[0].motor_value->target_speed_rpm	=	-	xiebo_1	(shoot_speed,3);
	Shoot_Motor[1].motor_value->target_speed_rpm	=		xiebo_1	(shoot_speed,3);


	pid_caculate(&Shoot_Motor[0].Motor_PID_Speed,		Shoot_Motor[0].motor_value->speed_rpm,		Shoot_Motor[0].motor_value->target_speed_rpm);
	pid_caculate(&Shoot_Motor[1].Motor_PID_Speed,		Shoot_Motor[1].motor_value->speed_rpm,		Shoot_Motor[1].motor_value->target_speed_rpm);
	
	set_chassis_current(&hcan1,
									Shoot_Motor[0].Motor_PID_Speed.out,
									Shoot_Motor[1].Motor_PID_Speed.out,
									0,
									0);
	C_Board.shoot	=	bodan_speed;
	bodan_speed	=	0;
}


void Shoot_Mode_Choose(void){
//////////////////B键选择发射机构开启/////////////
	if(IF_KEY_PRESSED_B)
	{
		if(b_b==0)
		{
			if(actShoot	!=	SHOOT_SLOW)
			{
				actShoot = SHOOT_SLOW;
			}else{
				actShoot = SHOOT_NORMAL;
			}
			b_b	=	1;
		}
	}else{
		b_b	=	0;
	}
//////////////////H键选择发射机构单发/////////////
	if(IF_KEY_PRESSED_R)
	{
		if(b_r==0)
		{
			if(actShoot!=SHOOT_SINGLE)
			{
				actShoot = SHOOT_SINGLE;
			}else{
				actShoot = SHOOT_NORMAL;
			}
			b_r	=	1;
		}
	}else{
		b_r	=	0;
	}  
//////////////////自定义/////////////
}


void Shoot_Key_Ctrl(void){
switch(actShoot) {
//////////////////自定义/////////////	
    case SHOOT_NORMAL:
			C_Board.mode	=	1;
			shoot_speed = 6600;
		if(IF_MOUSE_PRESSED_LEFT)
	{
			bodan_speed	=	2;
	}
        break;	
//////////////////自定义/////////////	
    case SHOOT_SINGLE:
			C_Board.mode	=	2;
			shoot_speed = 6600;
		if(IF_MOUSE_PRESSED_LEFT)
	{
		if(b_left==0)
		{
			bodan_speed	=	1;
			b_left	=	1;
		}
	}else{
		b_left	=	0;
	}			

        break;
//////////////////自定义/////////////				
    case SHOOT_SLOW:	
			C_Board.mode	=	0;
		shoot_speed = 0;			
		bodan_speed	=	0;
        break;
    default:
        break;
    }
}


int number_xiebo =0;
int number_flag =0;
int xiebo_1(int speed, int bu){
	if(number_xiebo	<	speed ){
		number_flag=1;
		number_xiebo+=bu;
	}else if(number_xiebo>speed){
		number_flag=-1;
		number_xiebo-=bu;
	}
	
	if(number_xiebo	>	speed &&	number_flag	==	1		){number_xiebo	=	speed;}
	if(number_xiebo	<	speed &&	number_flag	==	-1	){number_xiebo	=	speed;}
	
	return number_xiebo;
}
