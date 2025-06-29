#ifndef myparam_h
#define myparam_h
#include "main.h"

/***********************������Ϣ****************************************/

#define F_4   1      //ѡ�񼸺Ų�������    


#define CHASSIS_DECELE_RATIO  19		//���ٱ�
#define LENGTH_A 160         //mm
#define LENGTH_B 160         //mm
#define LENGTH_M 136			//mm,������
#define WHEEL_PERIMETER 153  //mm//ֱ��
#define WHEEL_WHERE  0.24  //m//���������ľ���
/***********************YAW����̨���������ض�ֵ******************/
#ifdef F_5
	#define GIMBAL_YAW_ENCODER_MIDDLE2 60		//���̺���̨������ͬ1��ָ��y
	#define GIMBAL_YAW_ENCODER_MIDDLE1 4156		//���̺���̨������ͬ2��ָ��-y
	#define GIMBAL_YAW_ENCODER_NINETY2 2108		//���̺���̨����90�㣬ָ��+90��
	#define GIMBAL_YAW_ENCODER_NINETY1 6204		//���̺���̨����90�㣬ָ��-90��
	#define GIMBAL_YAW_ENCODER_FORTYFIVE3 1084	//���̺���̨����45��1��ָ��45��
	#define GIMBAL_YAW_ENCODER_FORTYFIVE4 3132	//���̺���̨����45��2��ָ��135��
	#define GIMBAL_YAW_ENCODER_FORTYFIVE1 5180	//���̺���̨����45��3��ָ��-135��
	#define GIMBAL_YAW_ENCODER_FORTYFIVE2 7228	//���̺���̨����45��4��ָ��-45��
#elif F_4   //ȫ���ֲ���
	#define GIMBAL_YAW_ENCODER_MIDDLE1 478	//���̺���̨������ͬ1��ָ��y		
	#define GIMBAL_YAW_ENCODER_MIDDLE2 4536		//���̺���̨������ͬ2��ָ��-y
	#define GIMBAL_YAW_ENCODER_NINETY1 	6614	//���̺���̨����90�㣬ָ��+90��        
	#define GIMBAL_YAW_ENCODER_NINETY2 	2509	//���̺���̨����90�㣬ָ��-90�� 
	#define GIMBAL_YAW_ENCODER_FORTYFIVE1 1464	//���̺���̨����45��1��ָ��45��  
	#define GIMBAL_YAW_ENCODER_FORTYFIVE2 3495	//���̺���̨����45��2��ָ��135��	
	#define GIMBAL_YAW_ENCODER_FORTYFIVE3 5541	//���̺���̨����45��3��ָ��-135�� 
	#define GIMBAL_YAW_ENCODER_FORTYFIVE4 7613	//���̺���̨����45��4��ָ��-45��	
	#define GIMBAL_YAW_X 0.0 //Ĭ��������
#elif F_3
	#define GIMBAL_YAW_ENCODER_MIDDLE1 3183		//���̺���̨������ͬ1��ָ��y
	#define GIMBAL_YAW_ENCODER_MIDDLE2 7279		//���̺���̨������ͬ2��ָ��-y
	#define GIMBAL_YAW_ENCODER_NINETY1 5231		//���̺���̨����90�㣬ָ��+90��
	#define GIMBAL_YAW_ENCODER_NINETY2 1135		//���̺���̨����90�㣬ָ��-90��
	#define GIMBAL_YAW_ENCODER_FORTYFIVE1 377	//���̺���̨����45��1��ָ��45��
	#define GIMBAL_YAW_ENCODER_FORTYFIVE2 2340	//���̺���̨����45��2��ָ��135��
	#define GIMBAL_YAW_ENCODER_FORTYFIVE3 4648	//���̺���̨����45��3��ָ��-135��
	#define GIMBAL_YAW_ENCODER_FORTYFIVE4 6561	//���̺���̨����45��4��ָ��-45��
#endif
/**********����ģʽ�¸�����������Ƶ�����С  ***************/
	//��ͨ�Ǹ�����̨��������
	#define NOMOAL_CHASSIS_MAX1 20000
	#define NOMOAL_CHASSIS_MAX2 20000
	#define NOMOAL_CHASSIS_MAX3 20000
	#define NOMOAL_CHASSIS_MAX4 20000
	//���·Ǹ�����̨��������
	#define CLIMBING_CHASSIS_MAX1 20000
	#define CLIMBING_CHASSIS_MAX2 20000
	#define CLIMBING_CHASSIS_MAX3 20000
	#define CLIMBING_CHASSIS_MAX4 20000
	//��ͨ������̨��������
	#define NOMAL_FOLLOW_CHASSIS_MAX1 20000
	#define NOMAL_FOLLOW_CHASSIS_MAX2 20000
	#define NOMAL_FOLLOW_CHASSIS_MAX3 20000
	#define NOMAL_FOLLOW_CHASSIS_MAX4 20000
	//���¸�����̨��������
	#define CLIMBING_FOLLOW_CHASSIS_MAX1 20000
	#define CLIMBING_FOLLOW_CHASSIS_MAX2 20000
	#define CLIMBING_FOLLOW_CHASSIS_MAX3 20000
	#define CLIMBING_FOLLOW_CHASSIS_MAX4 20000
	//��ͨС����/Ťƨ������
	#define NOMAL_GYRO_CHASSIS_MAX1 20000
	#define NOMAL_GYRO_CHASSIS_MAX2 20000
	#define NOMAL_GYRO_CHASSIS_MAX3 20000
	#define NOMAL_GYRO_CHASSIS_MAX4 20000 
	//����С����/Ťƨ������
	#define CLIMBING_GYRO_CHASSIS_MAX1 20000
	#define CLIMBING_GYRO_CHASSIS_MAX2 20000
	#define CLIMBING_GYRO_CHASSIS_MAX3 20000
	#define CLIMBING_GYRO_CHASSIS_MAX4 20000 


/**********************************��̨��Ϣ****************************************/
	/***********************Pitch�ᡢYAW����̨��������λ****************************/
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
			
/********************ң����/���̲���****************************/
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VX 300.0f	//���̸�����̨ģʽ������vx  Խ��������ԽС
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VY 300.0f	//���̸�����̨ģʽ������vy  Խ��������ԽС


#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX 300.0f	//���̲�������̨ģʽ������vx  Խ��������ԽС
#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY 300.0f	//���̲�������̨ģʽ������vy  Խ��������ԽС


#define SENSITIVITY_REMOTE_GIMBAL_YAW 50.0f		//��̨������yaw�ᣬԽ��������ԽС
#define SENSITIVITY_REMOTE_GIMBAL_PITCH 50.0f		//��̨������pitch�ᣬԽ��������ԽС

#define SENSITIVITY_REMOTE_GIMBAL_YAW_IMU 1140.0f		//��̨������yaw�ᣬԽ��������ԽС
#define SENSITIVITY_REMOTE_GIMBAL_PITCH_IMU 1140.0f		//��̨������pitch�ᣬԽ��������ԽС

/***********************�Ӿ�������*************************************/

#define SENSITIVITY_VISION_GIMBAL_YAW_ENCODER 700.0f		//��̨������yaw�ᣬԽ��������ԽС
#define SENSITIVITY_VISION_GIMBAL_PITCH_ENCODER 700.0f		//��̨������pitch�ᣬԽ��������ԽС

/***************************������Ϣ****************************************/
			/******************���̵������****************/
			#define   REVOLVER_PID_POSITION_OUTMAX1       1000
			#define   REVOLVER_PID_POSITION_IMAX1         500
			#define   REVOLVER_PID_SPEED_OUTMAX2    20000  
			#define   REVOLVER_PID_SPEED_IMAX2      10000
			/******************����Ӳ���ߴ�******************/
			#define REVOL_SPEED_RATIO   2160       //�����һ��תһȦ,2160ת��ת��,60*36,����Ƶ�ٳ��Բ��̸����Ϳɵ���Ӧ��Ƶ�µ�ת��
			#define 	REVOL_SPEED_GRID      12			//���̸���
			#define    	AN_BULLET         (24576.0f)		//�����ӵ����λ������ֵ(���ֵ�ò�Ѽ)
      
			#define    	BUFF_R 1000
			

#endif


