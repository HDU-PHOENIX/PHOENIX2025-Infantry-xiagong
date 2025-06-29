#include "pid.h"
//#include "arm_math.h"
#include "math.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

//static float VariableIntegralCoefficient(float error, float absmax, float absmin);
/*参数初始化--------------------------------------------------------------*/
void pid_param_init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout,  float I_Separation, float Dead_Zone, float gama, int angle_max, int angle_min)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    if(fabs(pid->Dead_Zone) < 1e-5)
        pid->Dead_Zone = 0;
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->I_Separation = I_Separation;
    pid->Dead_Zone = Dead_Zone;
    pid->gama = gama;
    pid->angle_max = angle_max;
    pid->angle_min = angle_min;
}

float pid_caculate(PidTypeDef *pid, const float ref, const float set)
{
  	uint8_t index;
	if (pid == NULL||isnan(set)||isnan(ref)||isnan(pid->out))
	{
		pid->out = 0;
			return 0.0f;
		
	}
	pid->error[0] = set - ref;

	if((pid->angle_max!=pid->angle_min)&&pid->angle_min==0)
	{
		if( pid->error[0]>(pid->angle_max+pid->angle_min)/2)
			 pid->error[0]-=(pid->angle_max+pid->angle_min);
		else if( pid->error[0]<-(pid->angle_max+pid->angle_min)/2)
			 pid->error[0]+=(pid->angle_max+pid->angle_min);
	}else if(pid->angle_max == (-pid->angle_min)){
		if(pid->error[0]>pid->angle_max){
			pid->error[0]=pid->error[0]-2*pid->angle_max;
		}else if(pid->error[0]<pid->angle_min){
			pid->error[0]=pid->error[0]-2*pid->angle_min;
		}
	
	}
		if(fabs(pid->error[0])<pid->Dead_Zone){
				pid->error[0]=0;
		}
			
    if(fabs(pid->error[0])>pid->I_Separation)//误差过大，采用积分分离
    {
    	index=0;
    }
    else
    {
    	index=1;
    }
		
	pid->set = set;
	pid->fdb = ref;

    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout = (pid->Iout + pid->Ki * pid->error[0])*index;
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout=pid->Kd*(1- pid-> gama)*(pid->Dbuf[0])+pid-> gama* pid-> lastdout;
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid-> lastdout=pid->Dout;
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/*中途更改参数设定(调试)------------------------------------------------------------*/
void pid_reset(PidTypeDef	*pid, float PID[3])
{
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
}

/*pid结构体初始化，每一个pid参数需要调用一次-----------------------------------------------------*/
//void pid_init(PidTypeDef *pid)
//{
//    pid->f_param_init = pid_param_init;
//    pid->f_cal_pid = pid_caculate;
//    pid->f_reset_pid = pid_reset;
//}




