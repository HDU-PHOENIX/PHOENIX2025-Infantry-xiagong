#ifndef GIMBALTASKH
#define GIMBALTASKH

#include "main.h"
#include "type.h"

#define Pitch_max  40  // �Ƕ� //����
#define Pitch_min  -25  // �Ƕ� //����
/* ��̨����ģʽ:
   
   ��ͨ             	NORMAL
   ��ͷ180��             AROUND
   ���             	BUFF
   ����,pitchˮƽ   	LEVEL
   ��еģʽpitcḩͷ	HIGH
   ����Ťͷ90��          TURN
*/
typedef enum
{
	GIMBAL_NORMAL  					= 0,			//����ģʽ,����ģʽѡ��
	GIMBAL_TURN_RIGHT  			= 1,			//��ת90���ͷ
	GIMBAL_CHASSIS_FOLLOW   = 2,			//
	GIMBAL_LEVEL   					= 3,			//���ֿ���,��̨ˮƽ
	GIMBAL_MANUAL  					= 4,			//�ֶ����ģʽ
	GIMBAL_SM_BUFF 					= 5,			//С��
	GIMBAL_TURN_LEFT    		= 7,			//��ת90��Ťͷ
	GIMBAL_AUTO    					= 8,			//����
	GIMBAL_BASE    					= 9,			//��ͷ�������
	GIMBAL_BUFF 						=	10,			//��С��
	GIMBAL_BUFFBUFF 				= 11,	
	GIMBAL_PREDICT   				=	12,  		//Ԥ��ģʽ�е�����
	GIMBAL_GYROSCOPE 				=	13,			//С����
	GIMBAL_SLOW							= 14			//��Ĭ
}eGimbalAction;
extern eGimbalAction  actGimbal;

typedef struct  //�Ӿ�Ŀ���ٶȲ���
{
  int delay_cnt;//����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
  int freq;
  int last_time;//�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
  float last_position;//�ϸ�Ŀ��Ƕ�
  float speed;//�ٶ�
  float last_speed;//�ϴ��ٶ�
  float processed_speed;//�ٶȼ�����
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
void Gimbal_Control_Yaw(void);
void Gimbal_Control_Pitch(void);
void Gimbal_Control_Yaw_Auto(void);
void Gimbal_Control_Pitch_Auto(void);
void Gimbal_Single_Loop_Out(void);
void Gimbal_Vision(void);


void GetEnvironmentGimbalMode(void);	//���ó�����������
void GIMBAL_Mode_Choose(void);  			//��̨����ģʽѡ��,������Ӧ/
void GIMBAL_Key_Ctrl(void);     			//���̿�����̨ģʽ
void Gimbal_KeyControl_Yaw(void);
void Gimbal_KeyControl_Pitch(void);
#endif

