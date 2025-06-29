#ifndef __bsp_pid
#define __bsp_pid
#include "type.h"
#include "stm32f4xx_hal.h"
//#include "main.h"



//void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
//fp32 PID_Calc(PidTypeDef *pid, const fp32 ref, const fp32 set);
void pid_param_init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout,  float I_Separation, float Dead_Zone, float gama, int angle_max, int angle_min);
float pid_caculate(PidTypeDef *pid, const float ref, const float set);
void PID_clear(PidTypeDef *pid);
void pid_reset(PidTypeDef	*pid, fp32 PID[3]);
//void pid_init(PidTypeDef *pid);
//fp32 PID_Calc_Angle(PidTypeDef *pid, fp32 ref, fp32 set);
#endif
