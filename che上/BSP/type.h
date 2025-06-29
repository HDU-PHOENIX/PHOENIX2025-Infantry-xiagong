#ifndef __mytype
#define __mytype
//#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>

typedef float fp32;
typedef double fp64;

extern uint16_t CAN_RX_data_Frequent;
extern uint16_t CAN_RX_data_add ; 
extern uint16_t chassis_task_frequent;
extern uint16_t chassis_task_add;
extern uint16_t gimbal_task_frequent;
extern uint16_t gimbal_task_add;
extern uint16_t vision_frequent;
extern uint16_t vision_add;
extern uint16_t shoot_task_frequent;
extern uint16_t shoot_task_add;
extern uint16_t IMU_task_frequent;
extern uint16_t IMU_task_add;
extern uint16_t cap_add;
extern uint16_t cap_frequent;

typedef enum
{
	Starting=0,
	Running=1,
}SYSTEMVALUE;

typedef struct PidTypeDef
	{
    float Dead_Zone; //误差死区阈值
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set; //设定值
    float fdb; //反馈值

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
    float angle_max;
    float angle_min;	//角度相邻值 如在一个圆内，0°和360°相邻，则max=360，min=0
    					//			在一个电机内 0和8192相邻，则max=8192，min=0
    float I_Separation; //积分分离阈值
    float gama;			//微分先行滤波系数
    float lastdout;		//上一次微分输出
		
//	void ( *f_param_init)(struct PidTypeDef *pid,  //PID参数初始化
//		uint8_t mode,
//		fp32 PID[3],
//		fp32 max_out,
//		fp32 max_iout,
//		float I_Separation,
//		float Dead_Zone,
//		float gama,
//		int angle_max,
//		int angle_min
//	);
//	fp32 (*f_cal_pid)(struct PidTypeDef *pid, const fp32 ref, const fp32 set);   //pid计算
//						  //第一个参数为pid类输入参数，
//	void (*f_reset_pid)(struct PidTypeDef	*pid,fp32 PID[3]);
		
}PidTypeDef;

//void pid_init(PidTypeDef* pid);


typedef uint8_t 	u8;
typedef uint16_t 	u16;
typedef uint32_t 	u32;

typedef int8_t 		s8;
typedef int16_t 	s16;
typedef int32_t		s32;

typedef volatile uint8_t 	vu8;
typedef volatile uint16_t 	vu16;
typedef volatile uint32_t 	vu32;

typedef volatile int8_t 	vs8;
typedef volatile int16_t 	vs16;
typedef volatile int32_t	vs32;


typedef union{
	s8 		s8_fmt[8];	//for angle and omega
	u8 		u8_fmt[8];	//for angle and omega
	char 	ch_fmt[8];	//
	s16		s16_fmt[4];
	u16		u16_fmt[4];
	s32 	s32_fmt[2];
	u32 	u32_fmt[2];
	float 	f32_fmt[2];
	double 	d64_fmt;
}data_convert_ut;	//for diaobi gyro




typedef struct
{
	uint16_t RX_add;
	uint16_t RX_Frequent;
	
}rx_data;


typedef struct{
	int16_t	 	speed_rpm;
	int16_t   	target_speed_rpm;
	float  		real_current;
	int16_t  	given_current;
	uint8_t  	hall;
	volatile    int16_t 	angle;				//abs angle range:[0,8191]
	float       target_angle;		//目标角度
	uint16_t 	last_angle;			//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	u8			buf_idx;
	u16			angle_buf[5];
	u16			fited_angle;
	u32			msg_cnt;
	int16_t    	speed_dp10ms;		//机械角度/10ms   机械角度范围[0,8191]
														//转每10ms=speed_dp10ms/8192
	int16_t    	target_speed_dp10ms;
	uint16_t    last_angle_pre10ms;
    rx_data RX_data;

	float position,velocity,torque;	//三个电机的位置、速度、转矩解析存储	
	int16_t Error;
}moto_measure_t;//电机返回值的结构体


typedef	__packed struct{
	
	uint16_t x_int,	y_int,	w_int;
	uint8_t	mode;
	uint8_t	shoot;
	float x,y,w;
}C_Board_measure_t;



typedef struct
	{
	int ID;
	moto_measure_t* motor_value;
	PidTypeDef Motor_PID_Position;
	PidTypeDef Motor_PID_Speed;
	rx_data RX_data;
}Motortype;
	



typedef enum
{
	showno,
	showIMU,
	showMotor,
	showRc,
	showPC,
	showMotorSet,
	showJY901,


	shownothing,

}Oledshow_Typedef;



typedef enum
{
    PID_POSITION = 0,//位置式PID
    PID_DELTA = 1, //增量式PID
}PID_MODE;



typedef enum
{
	CAN_Moto1_ID = 0x201,
	CAN_Moto2_ID = 0x202,
	CAN_Moto3_ID = 0x203,
	CAN_Moto4_ID = 0x204,

	CAN_Moto5_ID = 0x205,
	CAN_Moto6_ID = 0x206,
	CAN_Moto7_ID = 0x207,
	CAN_Moto8_ID = 0x208,

	//6020电机接收报文stdID：0x2FF,设置电调ID为5-7。反馈报文为：5：0x209 6:0x20A 7:0x20B
	CAN_Moto9_ID = 0x209,
	CAN_Moto10_ID = 0x20A,
	CAN_Moto11_ID = 0x20B,
	
	CAN_SuperCap_ID = 0x210,
	
	Moto_Yaw_4310 = 0x07,
	Moto_Pitch_4310 = 0x09,
	
		C_Board_ID = 	0x105,
	C_Board_ID1 = 	0x104,
	
}CAN_Message_ID;



typedef enum
{
    Pen_Clear = 0x00,
    Pen_Write = 0x01,
    Pen_Inversion = 0x02,
}Pen_Typedef;
typedef enum
{
	key_no = 0x00,
	key_up =0x01,
	key_down=0x02,
	key_left=0x03,
	key_right=0x04,
	key_middle=0x05,
}KeyValue_Typedef;

typedef enum
{
	key_no_pres = 0x00,
	key_up_pres =0x01,
	key_down_pres=0x02,
	key_left_pres=0x03,
	key_right_pres=0x04,
	key_middle_pres=0x05,
}KeyPres_Typedef;


#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB

typedef enum
{
	KEYBOARD,
	REMOTE,
	FUWEI,
}ControlModeType;
extern ControlModeType ControlMode;

#endif
