#ifndef CHASSIStaskH
#define CHASSIStaskH
#include "main.h"
#include "type.h"
typedef enum
{
    CHASSIS_FOLLOW_GIMBAL = 0,	//���̸�����������
    CHASSIS_GYROSCOPE = 1,			//С����ģʽ
    CHASSIS_NORMAL   = 2,//���̲�������̨����
    CHASSIS_CORGI    = 3,//Ťƨ��ģʽ
    CHASSIS_ROSHAN   = 4,//���ģʽ
    CHASSIS_SLOW     = 5,//��������ģʽ
    CHASSIS_SZUPUP   = 6,//����ģʽ
    CHASSIS_MISS     = 7,//�Զ�����ģʽ
    CHASSIS_PISA     = 8,//45��ģʽ
	CHASSIS_FOLLOW_GIMBAL_45 = 9,	//���̸�����������45��ģʽ
	CHASSIS_FLY=10,   	 //����ģʽ
	CHASSIS_FOLLOW_GIMBAL_90 = 11,
    CHASSIS_GYROSCOPE_BIANSU=12//����С����
} eChassisAction;
extern eChassisAction actChassis;

extern ControlModeType ControlMode;
//����ģʽѡ��
typedef enum
{
    CHASSIS_MECH_MODE = 0,//��е
    CHASSIS_GYRO_MODE = 1,//������

} eChassisCtrlMode;
extern eChassisCtrlMode  modeChassis;

typedef struct
{
    float vx;
    float vy;
    float vw;
} Chassis_Speed;

void ChassisFun(void const * argument);

extern Chassis_Speed absolute_chassis_speed;
extern float mintemp1;
extern float text_speed;
extern Motortype Chassis_Motor[4];

void mecanum_calc(Chassis_Speed *speed, int16_t* out_speed);
void Mecanum_Set_Motor_Speed(int16_t*out_speed,moto_measure_t* Motor );
void Absolute_Cal(Chassis_Speed* absolute_speed, float angle )	;
void RemoteControlChassis(void);
void CHASSIS_Single_Loop_Out(void);
void CHASSIS_InitArgument(void);
float Find_Y_AnglePNY(void);
float FindMinAnglePNY(void);
float FindMinAngleFortyFive(void);
float FindMinAngleHUNHAEDFortyFive(void);
void Chassis_Mode_Choose(void);
/*****************����ģʽ*************************/
static void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp_inc, int16_t sMoveRamp_dec );
void Chassis_Mouse_Move_Calculate_45( void );
void Chassis_Mouse_Move_Calculate_135(void);
void CHAS_Key_Ctrl(void);
void Chassis_Mouse_Move_Calculate( void );
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec );

/*****************���̹���*************************/
void Chassis_Power_Limit(void) ;
/****��������****/
extern float text_3508[4];
//fp32 motor_text1[4];
extern uint16_t chassis_task_add;
#endif



