#include "chassis_task.h"
#include "type.h"
#include "judge.h"
#include "param.h"
#include "arm_math.h"
//#include "task.h"
#include "dbus.h"
#include "BSP_CAN.h"
//#include "protect_task.h"
//#include "motor.h"
//#include "cap.h"
#include "cmsis_os.h"
#include "tim.h"
//#include "chassis_power_control.h"
#include "math.h"
#include "pid.h"
#include "gimbal_task.h"
#include "SuperPower.h"

eChassisAction actChassis=CHASSIS_NORMAL;   //Ĭ�ϵ��̲�������̨����
eChassisCtrlMode  modeChassis=CHASSIS_GYRO_MODE; //Ĭ��Ϊ������ģʽ����
ControlModeType ControlMode=REMOTE;
extern uint8_t cap_switch;
uint16_t chassis_power_limit=0;//�۲����
uint16_t Power6=0;//���Ա���
float remian_energer=0;
fp32 motor_text[4] = {0};
//fp32 motor2_pid_for_reset[3] = {0};
//fp32 motor3_pid_for_reset[3] = {0};
//fp32 motor4_pid_for_reset[3] = {0};
extern uint16_t chassis_task_add;
float schange= 0.0f;
float i =0.0f;

uint16_t chassis_task_add=0;
void ChassisFun(void const * argument) 
{
    portTickType currentTime;
    CHASSIS_InitArgument();
		SuperPower_Init();
	//��ʼȫ��0
//	float  joule_residue = 0;//ʣ�ཹ����������
//	uint16_t power_limit = 0;
//	static uint8_t step_cap = 0;
    while(1)
    {  
        currentTime = xTaskGetTickCount();//��ǰϵͳʱ�� 
        //chassis_task_add++;
        switch(ControlMode)
        { 
        case KEYBOARD:
        {
//            Chassis_Mode_Choose();     //���ó�����������
//            CHAS_Key_Ctrl();			//���̽���������˶��ٶ�
//            CHASSIS_Single_Loop_Out();					//�����˶��ٶȽ���������˶��ٶ�
            break;
        }
        case REMOTE:
        {    
//            if(rc.sw1 == 2 && rc.sw1==2){
//                actChassis=CHASSIS_FOLLOW_GIMBAL;
//						}	//���̸�����̨
//					else if(rc.sw1 == 2 && rc.sw2==1){
//               actChassis=CHASSIS_NORMAL;
//					}	//���̲�������̨
//            else if(rc.sw1 == 3 && rc.sw2==2){
//                actChassis=CHASSIS_SLOW;
//						}		//���ģʽ   //CHASSIS_SLOW  CHASSIS_GYROSCOPE
//            else if(rc.sw1 == 3 && rc.sw2==1){
//                actChassis=CHASSIS_GYROSCOPE;
//						}		//С����ģʽ                                                       
//            RemoteControlChassis();				//����ң�����ٶȵ������ٶ�
						Communication_Chassis();
            CHASSIS_Single_Loop_Out();						//�����˶��ٶȽ���������˶��ٶ�
						SuperPower_Tx();
            break;
        }
		case FUWEI:
		{
			break;
		}
        default:
            break;
        }
//		if(++step_cap == 3)
//		{
//			/*��ȡ��������*/			
//			power_limit = JUDGE_usGetPowerLimit();
//			chassis_power_limit=power_limit;
//			/*ʣ�ཹ������*/
//			joule_residue = JUDGE_fGetRemainEnergy();
//			remian_energer=joule_residue;
//			set_cap0(&hcan2, robot_status, cap_switch, joule_residue);
//			set_cap1(&hcan2, robot_status, cap_switch, power_limit);
//			step_cap = 0;
//		}
				
        vTaskDelayUntil(&currentTime, 2);//������ʱ
				
    }
}

/**
  * @brief  ���̲�����ʼ��
  * @param  void
  * @retval void
  * @attention û�м�I�������,ֻ��ϵͳ����ʱ����һ��
  */
Motortype Chassis_Motor[4];//�ĸ�������в����Ľṹ��


PidTypeDef Bodan_Pid_T;
PidTypeDef Bodan_Pid;
fp32 bodan_pid[3]={10,1,0.83};
fp32 bodan_pid_t[3]={0,0,0.0};

Chassis_Speed absolute_chassis_speed;//Ŀ���ٶ�
Chassis_Speed xiebo_chassis_speed;//б���ٶ�
Chassis_Speed limit_chassis_speed;//�����ٶ�

uint8_t shoot_m	=	0;
int	M_speed;
int16_t chassis_motor[4];		//�ĸ�����Ŀ��ת��
fp32 chassisnothing[3]= {0};

fp32 motorid_speed_pid[4][3] = {{10,1,0.1},{10,1,0.1},{10,1,0.1},{10,1,0.1}};

//4.99999987e-06 0 1.99999995e-05 //4.09999984e-06 0 5.00000056e-07
void CHASSIS_InitArgument()
{ 
		pid_param_init(&Bodan_Pid_T,PID_POSITION,	bodan_pid_t,200,     200,   0, 0,       0, 0,  0);
		pid_param_init(&Bodan_Pid,PID_POSITION,	bodan_pid,	10000,     10000,   100, 0,       0, 0,  0);//0.00400000019
    /******************���̵��PID*****************************************/
    for(uint8_t i=0;i<4;i++)
    {
        Chassis_Motor[i].ID=i+1;
        Chassis_Motor[i].motor_value=&moto_CAN1[i];

			pid_param_init(&(Chassis_Motor[i].Motor_PID_Position),PID_POSITION,chassisnothing,0,0,3e38,0,0,8192,.0);
			pid_param_init(&(Chassis_Motor[i].Motor_PID_Speed),      PID_POSITION,  motorid_speed_pid[i],16384,   16384,     400,          0,       0.1,    0,            0 ); //16384
    }

}


/**
  * @brief  ͨ�ſ��Ʒ�ʽ
  * @param  void
  * @retval void
  * @attention
  *��Ҫ����absolute_chassis_speed�����̵��˶��������������˶�ѧ����
  */
void Communication_Chassis(void) {//ң�ط���ֵ���������ƶ�
    /***********************************ȷ�������ĸ������Ŀ���ٶ�*****************************************/
    absolute_chassis_speed.vx = C_Board.x;
		absolute_chassis_speed.vy = C_Board.y;
		absolute_chassis_speed.vw = C_Board.w;
		//shoot_m										=	C_Board.shoot;
}
void Communication_Shoot(void){
		shoot_m										=	C_Board.shoot;
}


/**
* @brief  �������ͺ���
  * @param  void
  * @retval void
  * @attention
  *��Ҫ����PID������ĸ������CAN����
  */
double 		value_for_try=0;
float 		Chassis_power=80.0;
uint16_t	Buffer_energy=60;
uint16_t 	Chassis_power_limit	=	100;
float step_wt=0.004;
float step_xt=0.004;
float step_yt=0.004;
float limit_t =1;
float	danfa=0.11111111;
float	jifen=0.0;
void CHASSIS_Single_Loop_Out() 
{
	//��������
	Chassis_power	=						JUDGE_fGetChassisPower();
	Buffer_energy	=						JUDGE_fGetRemainEnergy();
	Chassis_power_limit		=		JUDGE_usGetPowerLimit();
	if(Chassis_power_limit	==	0){
		Chassis_power=0.0;
		Chassis_power_limit	=	60.0;
	}
	
	//��ν�����bug
	if(actChassis==CHASSIS_FOLLOW_GIMBAL||actChassis==CHASSIS_NORMAL)//���ָ����λ�ò��ڱ�������ȷ��λ��
	{
		value_for_try=-(GIMBAL_YAW_X	-	Gimbal_Motor_Yaw.motor_value->position);
		if(value_for_try	<	-3.141593){
				value_for_try	= 3.141593+value_for_try+3.141593;
		}else	if(value_for_try	>	3.141593){
				value_for_try	= -3.141593*2+value_for_try;
		}
	}

	//�������
        Absolute_Cal(&absolute_chassis_speed,value_for_try);
	//��ֵ�����õ�����PID

	//����Ŀ������
    
	Chassis_Motor[0].motor_value->target_speed_rpm=chassis_motor[0];
    //chassis_motor[0];
	Chassis_Motor[1].motor_value->target_speed_rpm=chassis_motor[1];
	Chassis_Motor[2].motor_value->target_speed_rpm=chassis_motor[2];
	Chassis_Motor[3].motor_value->target_speed_rpm=chassis_motor[3];

	Chassis_Motor[0].motor_value->speed_rpm=moto_CAN1[0].speed_rpm;
	Chassis_Motor[1].motor_value->speed_rpm=moto_CAN1[1].speed_rpm;
	Chassis_Motor[2].motor_value->speed_rpm=moto_CAN1[2].speed_rpm;
	Chassis_Motor[3].motor_value->speed_rpm=moto_CAN1[3].speed_rpm;
    /************************************���̵���ٶȻ�����*********************************************/

    pid_caculate(&Chassis_Motor[0].Motor_PID_Speed,Chassis_Motor[0].motor_value->speed_rpm,Chassis_Motor[0].motor_value->target_speed_rpm);
    pid_caculate(&Chassis_Motor[1].Motor_PID_Speed,Chassis_Motor[1].motor_value->speed_rpm,Chassis_Motor[1].motor_value->target_speed_rpm);
    pid_caculate(&Chassis_Motor[2].Motor_PID_Speed,Chassis_Motor[2].motor_value->speed_rpm,Chassis_Motor[2].motor_value->target_speed_rpm);
    pid_caculate(&Chassis_Motor[3].Motor_PID_Speed,Chassis_Motor[3].motor_value->speed_rpm,Chassis_Motor[3].motor_value->target_speed_rpm);//���һ��
		
		if(shoot_m	>	0	&&shoot_m	<	200){
			jifen	+=	0.002*moto_CAN1[4].speed_rpm/36.0*35.0/47.0/60;
			pid_caculate(&Bodan_Pid_T	,jifen,	danfa);
			M_speed	=	Bodan_Pid_T.out	+	3000;
		}else	if(shoot_m==0){
			Communication_Shoot();
			M_speed	=	0;
		}else{
			Communication_Shoot();
			M_speed	=	-1000;
		}
		if(jifen	>=	danfa){
			jifen=0;
			shoot_m	-=1;
		}
			pid_caculate(&Bodan_Pid		,moto_CAN1[4].speed_rpm,	M_speed);

		
    /************************************�������������͸����*********************************************/

	set_chassis_current(&hcan1,
									Chassis_Motor[0].Motor_PID_Speed.out,
									Chassis_Motor[1].Motor_PID_Speed.out,
									Chassis_Motor[2].Motor_PID_Speed.out,
									Chassis_Motor[3].Motor_PID_Speed.out);

	set_moto5678_current(&hcan1,Bodan_Pid.out,0,0,0);
									
}






/**
* @brief  ȫ�����˶�ѧ������
  * @param  void
  * @retval void
  * @attention
  *��Ҫ�����ĸ����ӵ�ת��
  */


float speed_gain = 1;
void mecanum_calc(Chassis_Speed *speed, int16_t* out_speed) {
    int16_t wheel_rpm[4];//instand for four wheel
    float wheel_rpm_ratio;
    wheel_rpm_ratio = 60.0f/(WHEEL_PERIMETER*3.141593f)*CHASSIS_DECELE_RATIO*1000 * speed_gain;
    wheel_rpm[0] = (   speed->vx - speed->vy + speed->vw * (WHEEL_WHERE))*wheel_rpm_ratio;//left//x��y�����ٶ�,w����ת���ٶ�
    wheel_rpm[1] = (   speed->vx + speed->vy + speed->vw * (WHEEL_WHERE))*wheel_rpm_ratio;//forward
    wheel_rpm[2] = (  -speed->vx + speed->vy + speed->vw * (WHEEL_WHERE))*wheel_rpm_ratio;//right
    wheel_rpm[3] = (  -speed->vx - speed->vy + speed->vw * (WHEEL_WHERE))*wheel_rpm_ratio;//back

    memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t));//copy the rpm to out_speed
}


/*
 * @param absolute_speed ����������Ҫ���ٶ�
 * @param angle ��̨����ڵ��̵ĽǶ�
 */
float angle_hd;

Chassis_Speed temp_speed;
void Absolute_Cal(Chassis_Speed* absolute_speed, float angle )	{
  angle_hd=angle;
  temp_speed.vw	=	absolute_speed->vw;																							//��ʼ�Ƕ�Ϊ0���ٿ��Ǹ����벻����ʱ��ͷ�ǲ��������һ��
  temp_speed.vx	= absolute_speed->vx	*	cos(angle_hd)	+	absolute_speed->vy	*	sin(angle_hd);//������һ��
  temp_speed.vy	= absolute_speed->vx	*	sin(angle_hd)	-	absolute_speed->vy	*	cos(angle_hd);//Ӧ�û������������

	chassis_speed_xiebo_s(&temp_speed,&xiebo_chassis_speed);
	
	//mecanum_calc(&temp_speed,chassis_motor);//��̨����̵�����ԵýǶ�
  mecanum_calc(&xiebo_chassis_speed,chassis_motor);//��̨����̵�����ԵýǶ�
	
	//��֤�������������ͷ���ƶ���������ͷת��90��ʱx�����ٶȴ�1��0��
	//y�����ٶȴ�0��1����֤�Ӿ������������
}



Chassis_Step step_l,step_s;
float	speed_s,speed_v,speed_w,speed_l;
void chassis_speed_xiebo_s(Chassis_Speed* target_speed,Chassis_Speed* xiebo_speed){
	float fen,bx,by;
	speed_s	=	sqrtf(powf(xiebo_speed->vx,2)											+	powf(xiebo_speed->vy	,2));
	speed_v	=	sqrtf(powf(target_speed->vx	-	xiebo_speed->vx,2)	+	powf(target_speed->vy	-	xiebo_speed->vy,2));
	
	speed_l	=	fabs(xiebo_speed->vw);
	speed_w	=	fabs(target_speed->vw	-	xiebo_speed->vw);
	
	//step_l.t	= 0.004	*		powf(Chassis_power/Chassis_power_limit,2.0);
		step_l.t	= 0.004	*		(60.0-Buffer_energy)/(60.0-40.0);
	if(speed_l	>	0.006){
		step_l.w	=	(0	-	xiebo_speed->vw	)	/	speed_l	*	step_l.t	*	4;
	}else{
		step_l.w	=	0;
	}
	
	if(speed_s	>	0.002){
		step_l.x	=	(0.0	-	xiebo_speed->vx)	/	speed_s	*	step_l.t;
		step_l.y	=	(0.0	-	xiebo_speed->vy)	/	speed_s	*	step_l.t;
	}else{
		step_l.x	=	0;
		step_l.y	=	0; 
	}
	
	step_s.t	=	0.004;
	float	speed_m	=	sqrtf(powf(target_speed->vx,2)	+	powf(target_speed->vy	,2));
	if((speed_m	>	0.0001)&&	(speed_s<speed_m)){
		fen	=powf(speed_s	/	speed_m,2.0);
	}else{
		fen	=	1;
	}
		
	if(speed_w	>	0.006){
		step_s.w	=	(target_speed->vw	-	xiebo_speed->vw)	/	speed_w	*	step_s.t	*	4	*fen;
	}else{
		step_s.w	=	0;
	}
	
	if(speed_v	>	0.002){
		step_s.x	=	(target_speed->vx	-	xiebo_speed->vx)	/	speed_v	*	step_s.t;
		step_s.y	=	(target_speed->vy	-	xiebo_speed->vy)	/	speed_v	*	step_s.t;
	}else{ 
		step_s.x	=	0;
		step_s.y	=	0;
	}
	
	xiebo_speed->vx	=	xiebo_speed->vx	+	step_l.x	+	step_s.x;
	xiebo_speed->vy	=	xiebo_speed->vy	+	step_l.y	+	step_s.y;
	xiebo_speed->vw =	xiebo_speed->vw	+	step_l.w	+	step_s.w;
	
	if(speed_l	>	0.002	&&	speed_s	>	0.002){
		bx	=		xiebo_speed->vw	/	500	*	xiebo_speed->vy;
		by	=	-	xiebo_speed->vw	/	500	*	xiebo_speed->vx;
	}else{
		bx	=	0;
		by	=	0;
	}
	xiebo_speed->vx	=	xiebo_speed->vx	+	bx;
	xiebo_speed->vy	=	xiebo_speed->vy	+	by;
	
	if((speed_m	<	0.002)&&	(speed_s<0.002)){
		xiebo_speed->vx	=	0;
		xiebo_speed->vy	=	0;
	}
	if(speed_l	<	0.006	&&	speed_w	<	0.006	){
		xiebo_speed->vw =	0;
	}
	if(	Buffer_energy	<=	15){
		
		Chassis_Motor[0].Motor_PID_Speed.Iout=0;
		Chassis_Motor[1].Motor_PID_Speed.Iout=0;
		Chassis_Motor[2].Motor_PID_Speed.Iout=0;
		Chassis_Motor[3].Motor_PID_Speed.Iout=0;
	}
}
