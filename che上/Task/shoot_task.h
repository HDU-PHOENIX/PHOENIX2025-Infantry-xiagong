#ifndef SHOOTTASKH
#define SHOOTTASKH

#include "main.h"
#include "type.h"


typedef enum
{
	SHOOT_NORMAL  					= 0,			//����ģʽ,����ģʽѡ��
	SHOOT_SINGLE 						= 1,			//����
	SHOOT_SLOW          		=	2				//��Ĭ

}eShootAction;
extern eShootAction  actShoot;


extern uint8_t 		bodan_speed;
extern Motortype 			Shoot_Motor[2];

extern float tryas;

void ShootFun(void const * argument);
void Shoot_InitArgument(void);
void RemoteControlShoot(void);
void Shoot_Mode_Choose(void);  			
void Shoot_Key_Ctrl(void);     			
void Shoot_Single_Loop_Out(void);
int xiebo_1(int speed, int bu);

#endif