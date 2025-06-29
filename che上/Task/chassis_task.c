#include "chassis_task.h"
#include "type.h"
//#include "judge.h"
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
#include "shoot_task.h"

eChassisAction actChassis=CHASSIS_NORMAL;   //Ĭ�ϵ��̲�������̨����
eChassisCtrlMode  modeChassis=CHASSIS_GYRO_MODE; //Ĭ��Ϊ������ģʽ����
ControlModeType ControlMode=REMOTE;
extern uint8_t cap_switch;
uint16_t chassis_power_limit=0;//�۲����
uint16_t Power6=0;//���Ա���
float remian_energer=0;
fp32 motor_text[4] = {0};
extern uint16_t chassis_task_add;
float schange= 0.0f;
float i =0.0f;
float mmm;
float mmmm;
uint16_t chassis_task_add=0;
void ChassisFun(void const * argument) 
{
    portTickType currentTime;
    CHASSIS_InitArgument();

    while(1)
    {  
        currentTime = xTaskGetTickCount();//��ǰϵͳʱ�� 
        //chassis_task_add++;
				if(rc.sw1 == 1 && rc.sw2 == 2){
					if(ControlMode!=KEYBOARD){
						actChassis=CHASSIS_NORMAL;
						actGimbal = GIMBAL_NORMAL;
						actShoot	=	SHOOT_NORMAL;
					}
					ControlMode	=	KEYBOARD;
				}else{
					ControlMode	=	REMOTE;
				}
        switch(ControlMode)
        { 
        case KEYBOARD:
        {
            Chassis_Mode_Choose();     //���ó�����������
            CHAS_Key_Ctrl();			//���̽���������˶��ٶ�
						C_Board.x	=	absolute_chassis_speed.vx;
						C_Board.y	=	absolute_chassis_speed.vy;
						C_Board.w	=	absolute_chassis_speed.vw;
						//C_Board.mode	=	1;
						send_C_Board_current(&hcan2,	&C_Board);
            break;
        }
        case REMOTE:
        {    
            if((rc.sw1 == 2) && (rc.sw2==3)){
                actChassis=CHASSIS_FOLLOW_GIMBAL;
							//���̸�����̨
						}else if((rc.sw1 == 2 && rc.sw2 == 2)||(rc.sw1 == 2 && rc.sw2 == 1)){
               actChassis=CHASSIS_NORMAL;
						//���̲�������̨
            }else if((rc.sw1 == 3) && (rc.sw2==2)){
                actChassis=CHASSIS_SLOW;
								//���ģʽ   //CHASSIS_SLOW  CHASSIS_GYROSCOPE
            }else if((rc.sw1 == 3 && rc.sw2==3)||(rc.sw1 == 3 && rc.sw2==1)){
                actChassis=CHASSIS_GYROSCOPE;
						}		//С����ģʽ    
						else{
								actChassis=CHASSIS_SLOW;
						}
            RemoteControlChassis();				//����ң�����ٶȵ������ٶ�
            //CHASSIS_Single_Loop_Out();						//�����˶��ٶȽ���������˶��ٶ�
						
						C_Board.x	=	absolute_chassis_speed.vx;
						C_Board.y	=	absolute_chassis_speed.vy;
						C_Board.w	=	absolute_chassis_speed.vw;
						//C_Board.mode	=	0;
						//C_Board.shoot	=	0;
						send_C_Board_current(&hcan2,	&C_Board);

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


PidTypeDef Chassis_Follow_PID;
Chassis_Speed absolute_chassis_speed;//Ŀ���ٶ�
int16_t chassis_motor[4];		//�ĸ�����Ŀ��ת��
fp32 chassisnothing[3]= {0};

fp32 motorid_speed_pid[4][3] = {{10,1,0.1},{10,1,0.1},{10,1,0.1},{10,1,0.1}};

fp32 Chassis_Follow_pid[3]= {5 ,0 ,0};			
//4.99999987e-06 0 1.99999995e-05 //4.09999984e-06 0 5.00000056e-07
void CHASSIS_InitArgument()
{ 
    //���̸���ר��
		pid_param_init(&Chassis_Follow_PID,PID_POSITION,Chassis_Follow_pid,	3.141593,     3.14593,   0.5, 0.009,       0, 0,  0);//0.00400000019
		Chassis_Follow_PID.angle_max 		= 		3.141593;
		Chassis_Follow_PID.angle_min 		= 		-3.141593;
    /******************���̵��PID*****************************************/
    for(uint8_t i=0;i<4;i++)
    {
        Chassis_Motor[i].ID=i+1;
        Chassis_Motor[i].motor_value=&moto_CAN1[i];

			pid_param_init(&(Chassis_Motor[i].Motor_PID_Speed),      PID_POSITION,  motorid_speed_pid[i],16384,   16384,     400,          0,       0.1,    0,            0 ); //16384
    }

}

/**
  * @brief  ����ѡ�����ģʽ
  * @param  void
  * @retval void
  * @attention ģʽѡ��,����ĳģʽ��ǵ�д�˳�����ͨģʽ���ж�
  * �ް������»�һֱ�����Զ�����ģʽ,����ģʽ�л���İ�����������ģʽ�л�ѡ��ģʽ
  */

eChassisAction actChassis_last = CHASSIS_NORMAL;
int number_clear=0;
uint8_t	b_f,b_shift;
void Chassis_Mode_Choose()
{
//////////////////F��ѡ����̸�����̨/////////////
    if(IF_KEY_PRESSED_F)
    {
			if(b_f	==	0){
				if(actChassis_last	==	CHASSIS_NORMAL){
					actChassis	=	CHASSIS_FOLLOW_GIMBAL;
				}else	if(actChassis_last	==	CHASSIS_FOLLOW_GIMBAL){
					actChassis	=	CHASSIS_NORMAL;
				}else if(actChassis_last	==	CHASSIS_GYROSCOPE){
					actChassis	=	CHASSIS_FOLLOW_GIMBAL;
				}
				b_f	=	1;
			}
    }else{
			b_f	=	0;
		}
/////////////////Shift��ѡ��С����/////////////////////
    if(IF_KEY_PRESSED_SHIFT)
    {
       if(b_shift	==	0){
				if(actChassis_last	!=	CHASSIS_GYROSCOPE){
					actChassis	=	CHASSIS_GYROSCOPE;
				}else{
					actChassis	=	CHASSIS_FOLLOW_GIMBAL;
				}
				b_shift	=	1;
			}
    }else{
			b_shift	=	0;
		}
		
		
		actChassis_last	=	actChassis;
}

/**
  * @brief  ң�������Ʒ�ʽ
  * @param  void
  * @retval void
  * @attention
  *��Ҫ����absolute_chassis_speed�����̵��˶��������������˶�ѧ����
  */
void RemoteControlChassis(void) {//ң�ط���ֵ���������ƶ�
    /***********************************ȷ�������ĸ������Ŀ���ٶ�*****************************************/
	mmm=Find_Y_AnglePNY();
	mmmm=pid_caculate(&Chassis_Follow_PID,	Find_Y_AnglePNY(),	Gimbal_Motor_Yaw.motor_value->position);
    switch(actChassis) {
    case CHASSIS_FOLLOW_GIMBAL://������̨
        absolute_chassis_speed.vx= -(float)rc.ch2/250;
        absolute_chassis_speed.vy=  (float)rc.ch1/250;//(flag_Y_dirction)
        absolute_chassis_speed.vw= pid_caculate(&Chassis_Follow_PID,	Find_Y_AnglePNY(),	Gimbal_Motor_Yaw.motor_value->position);//PIDʹ���̸�����̨�ٶȣ��ҵ��̺���̨��С�Ƕ�
        break;                           //pid���㣬����������pid_caculate��Ŀ��Ƕ�
    case CHASSIS_NORMAL://��������̨
        absolute_chassis_speed.vx=-(float)rc.ch2/250;  //�����Ȳ��ù�
        absolute_chassis_speed.vy= (float)rc.ch1/250;
        absolute_chassis_speed.vw=0;
        break;
    case CHASSIS_GYROSCOPE:		//С����ģʽ
        absolute_chassis_speed.vx=-(float)rc.ch2/250;
        absolute_chassis_speed.vy= (float)rc.ch1/250;
        absolute_chassis_speed.vw=3.141593*4;
//    if(rc.wheel>0)
//         absolute_chassis_speed.vw=3.141593*2;
//        else if(rc.wheel<0)
//             absolute_chassis_speed.vw=-3.141593*2;
        break;
	case CHASSIS_SLOW:		//��̬ģʽ
        absolute_chassis_speed.vx=0;
        absolute_chassis_speed.vy=0;
        absolute_chassis_speed.vw=0;
        break;
    default:
        break;
    }
}
/**
  * @brief  ���̿��Ƶ���ģʽ
  * @param  void
  * @retval void
  * @attention
  */

float speed=2.50;
void CHAS_Key_Ctrl(void) {
 /***********************************ȷ�������ĸ������Ŀ���ٶ�*****************************************/
    switch(actChassis) {
    case CHASSIS_FOLLOW_GIMBAL://������̨
        absolute_chassis_speed.vx= (IF_KEY_PRESSED_S	-	IF_KEY_PRESSED_W)*speed;
        absolute_chassis_speed.vy= (IF_KEY_PRESSED_D	-	IF_KEY_PRESSED_A)*speed;
        absolute_chassis_speed.vw= 	pid_caculate(&Chassis_Follow_PID,	Find_Y_AnglePNY(),	Gimbal_Motor_Yaw.motor_value->position);//PIDʹ���̸�����̨�ٶȣ��ҵ��̺���̨��С�Ƕ�
        break;                           //pid���㣬����������pid_caculate��Ŀ��Ƕ�
    case CHASSIS_NORMAL://��������̨
        absolute_chassis_speed.vx= (IF_KEY_PRESSED_S	-	IF_KEY_PRESSED_W)*speed;
        absolute_chassis_speed.vy= (IF_KEY_PRESSED_D	-	IF_KEY_PRESSED_A)*speed;
        absolute_chassis_speed.vw=0;
        break;
    case CHASSIS_GYROSCOPE:		//С����ģʽ
        absolute_chassis_speed.vx= (IF_KEY_PRESSED_S	-	IF_KEY_PRESSED_W)*speed;
        absolute_chassis_speed.vy= (IF_KEY_PRESSED_D	-	IF_KEY_PRESSED_A)*speed;
        absolute_chassis_speed.vw=3.141593*4;
//    if(rc.wheel>0)
//         absolute_chassis_speed.vw=3.141593*2;
//        else if(rc.wheel<0)
//             absolute_chassis_speed.vw=-3.141593*2;
        break;
	case CHASSIS_SLOW:		//��̬ģʽ
        absolute_chassis_speed.vx=0;
        absolute_chassis_speed.vy=0;
        absolute_chassis_speed.vw=0;
        break;
    default:
        break;
    }
}



/**
* @brief  �������ͺ���
  * @param  void
  * @retval void
  * @attention
  *��Ҫ����PID������ĸ������CAN����
  */
double value_for_try=0;
void CHASSIS_Single_Loop_Out() 
{
	//��ν�����bug
	if(actChassis==CHASSIS_FOLLOW_GIMBAL||actChassis==CHASSIS_NORMAL)//���ָ����λ�ò��ڱ�������ȷ��λ��
	{
		value_for_try=(GIMBAL_YAW_X	-	Gimbal_Motor_Yaw.motor_value->position);
		if(value_for_try	<	-3.141593){
				value_for_try	= 3.141593+value_for_try+3.141593;
		}else	if(value_for_try	>	3.141593){
				value_for_try	= -3.141593*2+value_for_try;
		}
	}

	//
	
	//�������
        Absolute_Cal(&absolute_chassis_speed,value_for_try);
//	//��ֵ�����õ�����PID

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
		
    /************************************�������������͸����*********************************************/
//	Send_cap_msg();	  				
//    chassis_power_control();
	set_chassis_current(&hcan1,
									Chassis_Motor[0].Motor_PID_Speed.out,
									Chassis_Motor[1].Motor_PID_Speed.out,
									Chassis_Motor[2].Motor_PID_Speed.out,
									Chassis_Motor[3].Motor_PID_Speed.out);
									
}






/**
* @brief  ȫ�����˶�ѧ������
  * @param  void
  * @retval void
  * @attention
  *��Ҫ�����ĸ����ӵ�ת��
  */

//int aa[4];
//int b;
float speed_gain = 1;
void mecanum_calc(Chassis_Speed *speed, int16_t* out_speed) {
    int16_t wheel_rpm[4];//instand for four wheel
    float wheel_rpm_ratio;
    wheel_rpm_ratio = 60.0f/(WHEEL_PERIMETER*3.14f)*CHASSIS_DECELE_RATIO*1000 * speed_gain;
//	b=wheel_rpm_ratio;
    wheel_rpm[0] = (   speed->vx - speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//left//x��y�����ٶ�,w����ת���ٶ�
    wheel_rpm[1] = (   speed->vx + speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//forward
    wheel_rpm[2] = (  -speed->vx + speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//right
    wheel_rpm[3] = (  -speed->vx - speed->vy + speed->vw * (LENGTH_A+LENGTH_B))*wheel_rpm_ratio;//back
//	for(int i=0;i<4;i++){
//		aa[i]=wheel_rpm[i];
//	}
    memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t));//copy the rpm to out_speed
}


/*
 * @param absolute_speed ����������Ҫ���ٶ�
 * @param angle ��̨����ڵ��̵ĽǶ�
 */
float angle_see;
float yaw_angle_see;
float cos_value=0;
float sin_value=0;
Chassis_Speed temp_speed;
void Absolute_Cal(Chassis_Speed* absolute_speed, float angle )	{
	yaw_angle_see=angle;
    float angle_hd=angle;
	angle_see=angle_hd;	
    temp_speed.vw=absolute_speed->vw;//��ʼ�Ƕ�Ϊ0���ٿ��Ǹ����벻����ʱ��ͷ�ǲ��������һ��
    temp_speed.vx= absolute_speed->vx*cos(angle_hd)+absolute_speed->vy*sin(angle_hd);//������һ��
    temp_speed.vy= absolute_speed->vx*sin(angle_hd)-absolute_speed->vy*cos(angle_hd);//Ӧ�û������������
	cos_value=cos(angle_hd);
	sin_value=sin(angle_hd);
    mecanum_calc(&temp_speed,chassis_motor);//��̨����̵�����ԵýǶ�
	//��֤�������������ͷ���ƶ���������ͷת��90��ʱx�����ٶȴ�1��0��
	//y�����ٶȴ�0��1����֤�Ӿ������������
}

//float last_position;
/**
  * @brief  �ҳ���45������Сƫ���
  * @param  void
  * @retval ƫ��ǣ��Ƕ���
  * @attention ͨ��ң����/����
  */
//float FindMinAngleFortyFive(void) {

//    float temp1=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_FORTYFIVE1;
//    float temp4=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_FORTYFIVE3;
//    float mintemp1,mintemp2;
//    if(temp1>4096)
//        temp1-=8192;
//    else if(temp1<-4096)
//        temp1+=8192;

//    if(temp4>4096)
//        temp4-=8192;
//    else if(temp4<-4096)
//        temp4+=8192;
//    mintemp2=(fabs(temp1)<fabs(temp4)?temp1:temp4);
//    return mintemp2;
//}
/**
  * @brief  �ҳ���135������Сƫ���
  * @param  void
  * @retval ƫ��ǣ��Ƕ���
  * @attention ͨ��ң����/����
  */
//float FindMinAngleHUNHAEDFortyFive(void) {

//    float temp1=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_FORTYFIVE2;
//    float temp4=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_FORTYFIVE4;
//    float mintemp2;
//    if(temp1>4096)
//        temp1-=8192;
//    else if(temp1<-4096)
//        temp1+=8192;

//    if(temp4>4096)
//        temp4-=8192;
//    else if(temp4<-4096)
//        temp4+=8192;
//    mintemp2=(fabs(temp1)<fabs(temp4)?temp1:temp4);
//    return mintemp2;
//}

/**
  * @brief  �ҳ���+-y����Сƫ���
  * @param  void
  * @retval ƫ��ǣ��Ƕ���
  * @attention ͨ��ң����/����
  */
//float FindMinAnglePNY(void) {
//    float temp1=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE1;
//    float temp2=Gimbal_MotorYaw.motor_value->angle-GIMBAL_YAW_ENCODER_MIDDLE2;
//	float mintemp1;
//    if(temp1>4096)
//        temp1-=8192;
//    else if(temp1<-4096)
//        temp1+=8192;
//    if(temp2>4096)
//        temp2-=8192;
//    else if(temp2<-4096)
//        temp2+=8192;
//	mintemp1 = (fabs(temp1) < fabs(temp2) ? temp1 : temp2);
//    return mintemp1;
//}


/**
  * @brief  �ҳ���+y����Сƫ���
  * @param  void
  * @retval ƫ��ǣ��Ƕ���
  * @attention ͨ��ң����/����
  */
    float mintemp1;
float Find_Y_AnglePNY(void)
{//###########################��ǰ�Ƕȼ�ȥĿ��Ƕ�    
		if(	fabs(GIMBAL_YAW_X	-	Gimbal_Motor_Yaw.motor_value->position)<3.141593/2){
			mintemp1=GIMBAL_YAW_X;
		}else	if(fabs(GIMBAL_YAW_X	-	Gimbal_Motor_Yaw.motor_value->position)>3.141593/2*3){
			mintemp1=GIMBAL_YAW_X;
		}else{
			mintemp1=GIMBAL_YAW_TX;
		}

    return mintemp1;
}

