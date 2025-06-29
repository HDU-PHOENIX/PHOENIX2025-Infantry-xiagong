#ifndef myparam_h
#define myparam_h
#include "main.h"

/***********************底盘信息****************************************/

#define F_4   1      //选择几号步兵代码    


#define CHASSIS_DECELE_RATIO  19		//减速比
#define LENGTH_A 160         //mm
#define LENGTH_B 160         //mm
#define LENGTH_M 136			//mm,麦轮用
#define WHEEL_PERIMETER 153  //mm//直径
#define WHEEL_WHERE  0.24  //m//轮子与中心距离
/***********************YAW轴云台编码器的特定值******************/
#ifdef F_5
	#define GIMBAL_YAW_ENCODER_MIDDLE2 60		//底盘和云台朝向相同1，指向＋y
	#define GIMBAL_YAW_ENCODER_MIDDLE1 4156		//底盘和云台朝向相同2，指向-y
	#define GIMBAL_YAW_ENCODER_NINETY2 2108		//底盘和云台朝向90°，指向+90°
	#define GIMBAL_YAW_ENCODER_NINETY1 6204		//底盘和云台朝向90°，指向-90°
	#define GIMBAL_YAW_ENCODER_FORTYFIVE3 1084	//底盘和云台朝向45°1，指向45°
	#define GIMBAL_YAW_ENCODER_FORTYFIVE4 3132	//底盘和云台朝向45°2，指向135°
	#define GIMBAL_YAW_ENCODER_FORTYFIVE1 5180	//底盘和云台朝向45°3，指向-135°
	#define GIMBAL_YAW_ENCODER_FORTYFIVE2 7228	//底盘和云台朝向45°4，指向-45°
#elif F_4   //全向轮步兵
	#define GIMBAL_YAW_ENCODER_MIDDLE1 478	//底盘和云台朝向相同1，指向＋y		
	#define GIMBAL_YAW_ENCODER_MIDDLE2 4536		//底盘和云台朝向相同2，指向-y
	#define GIMBAL_YAW_ENCODER_NINETY1 	6614	//底盘和云台朝向90°，指向+90°        
	#define GIMBAL_YAW_ENCODER_NINETY2 	2509	//底盘和云台朝向90°，指向-90° 
	#define GIMBAL_YAW_ENCODER_FORTYFIVE1 1464	//底盘和云台朝向45°1，指向45°  
	#define GIMBAL_YAW_ENCODER_FORTYFIVE2 3495	//底盘和云台朝向45°2，指向135°	
	#define GIMBAL_YAW_ENCODER_FORTYFIVE3 5541	//底盘和云台朝向45°3，指向-135° 
	#define GIMBAL_YAW_ENCODER_FORTYFIVE4 7613	//底盘和云台朝向45°4，指向-45°	
	#define GIMBAL_YAW_X 0.0 //默认正方向
#elif F_3
	#define GIMBAL_YAW_ENCODER_MIDDLE1 3183		//底盘和云台朝向相同1，指向＋y
	#define GIMBAL_YAW_ENCODER_MIDDLE2 7279		//底盘和云台朝向相同2，指向-y
	#define GIMBAL_YAW_ENCODER_NINETY1 5231		//底盘和云台朝向90°，指向+90°
	#define GIMBAL_YAW_ENCODER_NINETY2 1135		//底盘和云台朝向90°，指向-90°
	#define GIMBAL_YAW_ENCODER_FORTYFIVE1 377	//底盘和云台朝向45°1，指向45°
	#define GIMBAL_YAW_ENCODER_FORTYFIVE2 2340	//底盘和云台朝向45°2，指向135°
	#define GIMBAL_YAW_ENCODER_FORTYFIVE3 4648	//底盘和云台朝向45°3，指向-135°
	#define GIMBAL_YAW_ENCODER_FORTYFIVE4 6561	//底盘和云台朝向45°4，指向-45°
#endif
/**********各个模式下各个电机的限制电流大小  ***************/
	//普通非跟随云台底盘限流
	#define NOMOAL_CHASSIS_MAX1 20000
	#define NOMOAL_CHASSIS_MAX2 20000
	#define NOMOAL_CHASSIS_MAX3 20000
	#define NOMOAL_CHASSIS_MAX4 20000
	//爬坡非跟随云台底盘限流
	#define CLIMBING_CHASSIS_MAX1 20000
	#define CLIMBING_CHASSIS_MAX2 20000
	#define CLIMBING_CHASSIS_MAX3 20000
	#define CLIMBING_CHASSIS_MAX4 20000
	//普通跟随云台底盘限流
	#define NOMAL_FOLLOW_CHASSIS_MAX1 20000
	#define NOMAL_FOLLOW_CHASSIS_MAX2 20000
	#define NOMAL_FOLLOW_CHASSIS_MAX3 20000
	#define NOMAL_FOLLOW_CHASSIS_MAX4 20000
	//爬坡跟随云台底盘限流
	#define CLIMBING_FOLLOW_CHASSIS_MAX1 20000
	#define CLIMBING_FOLLOW_CHASSIS_MAX2 20000
	#define CLIMBING_FOLLOW_CHASSIS_MAX3 20000
	#define CLIMBING_FOLLOW_CHASSIS_MAX4 20000
	//普通小陀螺/扭屁股限流
	#define NOMAL_GYRO_CHASSIS_MAX1 20000
	#define NOMAL_GYRO_CHASSIS_MAX2 20000
	#define NOMAL_GYRO_CHASSIS_MAX3 20000
	#define NOMAL_GYRO_CHASSIS_MAX4 20000 
	//爬坡小陀螺/扭屁股限流
	#define CLIMBING_GYRO_CHASSIS_MAX1 20000
	#define CLIMBING_GYRO_CHASSIS_MAX2 20000
	#define CLIMBING_GYRO_CHASSIS_MAX3 20000
	#define CLIMBING_GYRO_CHASSIS_MAX4 20000 


/**********************************云台信息****************************************/
	/***********************Pitch轴、YAW轴云台编码器限位****************************/
	#ifdef F_3
			#define GIMBAL_PITCH_ENCODER_MAX 484    //up
			#define GIMBAL_PITCH_ENCODER_MIDDLE 7800  //5700
			#define GIMBAL_PITCH_ENCODER_MIN 6900     //down

	#elif F_4
			#define GIMBAL_PITCH_ENCODER_MAX 2500    //up
			#define GIMBAL_PITCH_ENCODER_MIDDLE 2918  //5700
			#define GIMBAL_PITCH_ENCODER_MIN 1050     //down
	#elif F_5
			#define GIMBAL_PITCH_ENCODER_MAX 2500    //up
			#define GIMBAL_PITCH_ENCODER_MIDDLE 1680  //5700
			#define GIMBAL_PITCH_ENCODER_MIN 1050     //down
	#endif
			
/********************遥控器/键盘参数****************************/
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VX 300.0f	//底盘跟随云台模式灵敏度vx  越大灵敏度越小
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VY 300.0f	//底盘跟随云台模式灵敏度vy  越大灵敏度越小


#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX 300.0f	//底盘不跟随云台模式灵敏度vx  越大灵敏度越小
#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY 300.0f	//底盘不跟随云台模式灵敏度vy  越大灵敏度越小


#define SENSITIVITY_REMOTE_GIMBAL_YAW 50.0f		//云台灵敏度yaw轴，越大灵敏度越小
#define SENSITIVITY_REMOTE_GIMBAL_PITCH 50.0f		//云台灵敏度pitch轴，越大灵敏度越小

#define SENSITIVITY_REMOTE_GIMBAL_YAW_IMU 1140.0f		//云台灵敏度yaw轴，越大灵敏度越小
#define SENSITIVITY_REMOTE_GIMBAL_PITCH_IMU 1140.0f		//云台灵敏度pitch轴，越大灵敏度越小

/***********************视觉灵敏度*************************************/

#define SENSITIVITY_VISION_GIMBAL_YAW_ENCODER 700.0f		//云台灵敏度yaw轴，越大灵敏度越小
#define SENSITIVITY_VISION_GIMBAL_PITCH_ENCODER 700.0f		//云台灵敏度pitch轴，越大灵敏度越小

/***************************拨盘信息****************************************/
			/******************拨盘电机限流****************/
			#define   REVOLVER_PID_POSITION_OUTMAX1       1000
			#define   REVOLVER_PID_POSITION_IMAX1         500
			#define   REVOLVER_PID_SPEED_OUTMAX2    20000  
			#define   REVOLVER_PID_SPEED_IMAX2      10000
			/******************拨盘硬件尺寸******************/
			#define REVOL_SPEED_RATIO   2160       //电机轴一秒转一圈,2160转子转速,60*36,乘射频再除以拨盘格数就可得相应射频下的转速
			#define 	REVOL_SPEED_GRID      12			//拨盘格数
			#define    	AN_BULLET         (24576.0f)		//单个子弹电机位置增加值(这个值得测鸭)
      
			#define    	BUFF_R 1000
			

#endif


