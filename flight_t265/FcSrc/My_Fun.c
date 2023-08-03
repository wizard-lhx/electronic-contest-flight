#include "Drv_PID.h"
#include "ANO_LX.h"
#include "Drv_RasPi.h"
#include "Drv_AnoOf.h"
#include "Ano_Math.h"
#include "Ano_DT_LX.h"

s16 position_x_set[12] = {0,  0,    310,  310,  80,   80,   300,  300,  80,   80,  300, 300};
s16 position_y_set[12] = {0,  -380, -380, -320, -320, -240, -240, -160, -160, -80, -80, 0};
s16 position_z_set[12] = {180,180,  180,  180,  180,  180,  180,  180,  180,  180, 180, 180};
float vector_flight_vel[2];
u8 Set_Position(u8 position_num)
{
	float world_vel_x = PID_Cal(&pos_x_pid, position_x_set[position_num], raspi.x);
	float world_vel_y = PID_Cal(&pos_y_pid, position_y_set[position_num], raspi.y);
	float world_vel_z = PID_Cal(&pos_z_pid, position_z_set[position_num], ano_of.of_alt_cm);

	float vector_world_vel[2] = {world_vel_x, world_vel_y};
	rot_vec_2(vector_world_vel, (float)my_sin(raspi.angle/100.0*ONE_PI/180.0), 
		(float)my_cos(raspi.angle/100.0*ONE_PI/180.0), vector_flight_vel);

	vector_flight_vel[0] = LIMIT(vector_flight_vel[0],-pos_x_pid.max_out,pos_x_pid.max_out);
	s16 flight_vel_x = (s16)vector_flight_vel[0];
	vector_flight_vel[1] = LIMIT(vector_flight_vel[1],-pos_y_pid.max_out,pos_y_pid.max_out);
	s16 flight_vel_y = (s16)vector_flight_vel[1];
	world_vel_z = LIMIT(world_vel_z,-pos_z_pid.max_out,pos_z_pid.max_out);
	s16 flight_vel_z = (s16)world_vel_z;
	dt.fun[0xf2].WTS = 1;

	rt_tar.st_data.vel_x = flight_vel_x;
	rt_tar.st_data.vel_y = flight_vel_y;
	rt_tar.st_data.vel_z = flight_vel_z;
	
	if(position_x_set[position_num] - raspi.x < 3 && position_y_set[position_num] - raspi.y < 3 && 
		position_z_set[position_num] - (s16)ano_of.of_alt_cm < 3 && position_x_set[position_num] - raspi.x > -3 && 
	position_y_set[position_num] - raspi.y > -3 && position_z_set[position_num] - (s16)ano_of.of_alt_cm > -3)
	{
		rt_tar.st_data.vel_x = 0;
		rt_tar.st_data.vel_y = 0;
		rt_tar.st_data.vel_z = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

float flight_wz;

u8 Set_Angle(s16 angle)
{
	flight_wz = PID_Cal(&angle_pid, angle, raspi.angle/100.0f);
	
	flight_wz = LIMIT(flight_wz,-angle_pid.max_out,angle_pid.max_out);
	
	rt_tar.st_data.yaw_dps = -((s16)flight_wz);
	
	if(angle - raspi.angle/100.0 < 3 && angle - raspi.angle/100.0 > -3)
	{
		flight_wz = 0;
		rt_tar.st_data.yaw_dps = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

u8 Take_Off(u32 height)
{
	rt_tar.st_data.vel_z = 30;
	if(ano_of.of_alt_cm > (height - 10))
	{
		rt_tar.st_data.vel_z = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

void Land(void)
{
	rt_tar.st_data.vel_z = -30;
}
