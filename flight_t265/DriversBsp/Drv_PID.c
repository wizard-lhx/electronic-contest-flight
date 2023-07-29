#include "Drv_PID.h"

pid_t pos_x_pid;
pid_t pos_y_pid;
pid_t pos_z_pid;
pid_t angle_pid;

void PID_Init(pid_t *pid, float kp, float ki, float kd, float max_iout, float max_out)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->max_iout = max_iout;
	pid->max_out = max_out;
	
	pid->pout=pid->iout=pid->dout=pid->error=pid->last_error=0.0f;
}

float PID_Cal(pid_t *pid, float set, float ref)
{
	pid->error = set - ref;
	
	pid->pout = pid->kp * pid->error;
	pid->iout += pid->ki * pid->error;
	pid->dout = pid->kd * (pid->error - pid->last_error);
	
	pid->last_error = pid->error;
	
	if(pid->iout > pid->max_iout)
	{
		pid->iout = pid->max_iout;
	}
	else if(pid->iout < -pid->max_iout)
	{
		pid->iout = -pid->max_iout;
	}
	
	pid->out = pid->pout + pid->iout + pid->dout;
	
	if(pid->out > pid->max_out)
	{
		pid->out = pid->max_out;
	}
	else if(pid->out < -pid->max_out)
	{
		pid->out = -pid->max_out;
	}
	
	return pid->out;
}
