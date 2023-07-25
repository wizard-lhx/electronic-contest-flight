#include "Drv_PID.h"
#include "ANO_LX.h"
#include "Drv_RasPi.h"
#include "Drv_AnoOf.h"
#include "Ano_Math.h"

s16 position_x_set[10] = {0, 50};
s16 position_y_set[10] = {0, 50};
s16 position_z_set[10] = {50,50};

u8 Set_Position(u8 position_num)
{
	float world_vel_x = PID_Cal(&pos_x_pid, position_x_set[position_num], raspi.x);
	float world_vel_y = PID_Cal(&pos_y_pid, position_y_set[position_num], raspi.y);
	float world_vel_z = PID_Cal(&pos_z_pid, position_z_set[position_num], ano_of.of_alt_cm);
	
	float vector_world_vx[2] = {world_vel_x, 0}; 
	float vector_flight_vx[2];
	rot_vec_2(vector_world_vx, (float)my_sin(raspi.angle/100.0*ONE_PI/180.0), 
		(float)my_cos(raspi.angle/100.0*ONE_PI/180.0), vector_flight_vx);
	
	float vector_world_vy[2] = {0, world_vel_y};
	float vector_flight_vy[2];
	rot_vec_2(vector_world_vy, (float)my_sin(raspi.angle/100.0*ONE_PI/180.0), 
		(float)my_cos(raspi.angle/100.0*ONE_PI/180.0), vector_flight_vy);

	s16 flight_vel_x = (s16)((s32)vector_flight_vx[0] + vector_flight_vy[0]);
	LIMIT(flight_vel_x,-pos_x_pid.max_out,pos_x_pid.max_out);
	s16 flight_vel_y = (s16)((s32)vector_flight_vx[1] + vector_flight_vy[1]);
	LIMIT(flight_vel_y,-pos_y_pid.max_out,pos_y_pid.max_out);
	
	rt_tar.st_data.vel_x = flight_vel_x;
	rt_tar.st_data.vel_y = flight_vel_y;
	rt_tar.st_data.vel_z = (s16)((s32)world_vel_z);
	
	if(rt_tar.st_data.vel_x < 3 && rt_tar.st_data.vel_y < 3 && rt_tar.st_data.vel_z < 3)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

u8 Set_Angle(s16 angle)
{
	float flight_wz = PID_Cal(&angle_pid, angle, raspi.angle);
	
	rt_tar.st_data.yaw_dps = flight_wz;
	
	if(rt_tar.st_data.yaw_dps < 3)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
