#include "Drv_PID.h"
#include "ANO_LX.h"
#include "Drv_RasPi.h"
#include "Drv_AnoOf.h"
#include "Ano_Math.h"
#include "Ano_DT_LX.h"

s16 position_x_set[10] = {0, 50, 50,  -50};
s16 position_y_set[10] = {0, 50, -50, 0  };
s16 position_z_set[10] = {50,100,100, 100};

u8 Set_Position(u8 position_num)
{
	float world_vel_x = PID_Cal(&pos_x_pid, position_x_set[position_num], raspi.x);
	float world_vel_y = PID_Cal(&pos_y_pid, position_y_set[position_num], raspi.y);
	float world_vel_z = PID_Cal(&pos_z_pid, position_z_set[position_num], ano_of.of_alt_cm);
	
//	vector3d_t vector_world_vel = {world_vel_x, world_vel_y, world_vel_z};
//	quaternion_t quaternion_flight = {fc_att_qua.st_data.w_x10000/10000.0f, fc_att_qua.st_data.x_x10000/10000.0f,
//			fc_att_qua.st_data.y_x10000/10000.0f, fc_att_qua.st_data.z_x10000/10000.0f};
	vector3d_t vector_flight_vel = {world_vel_x, world_vel_y, world_vel_z};

	vector_flight_vel.x = LIMIT(vector_flight_vel.x,-pos_x_pid.max_out,pos_x_pid.max_out);
	s16 flight_vel_x = (s16)vector_flight_vel.x;
	vector_flight_vel.y = LIMIT(vector_flight_vel.y,-pos_y_pid.max_out,pos_y_pid.max_out);
	s16 flight_vel_y = (s16)vector_flight_vel.y;
	vector_flight_vel.z = LIMIT(vector_flight_vel.z,-pos_z_pid.max_out,pos_z_pid.max_out);
	s16 flight_vel_z = (s16)vector_flight_vel.z;
	
	rt_tar.st_data.vel_x = flight_vel_x;
	rt_tar.st_data.vel_y = flight_vel_y;
	rt_tar.st_data.vel_z = flight_vel_z;
	
	if(rt_tar.st_data.vel_x < 3 && rt_tar.st_data.vel_y < 3 && rt_tar.st_data.vel_z < 3 && 
		rt_tar.st_data.vel_x > -3 && rt_tar.st_data.vel_y > -3 && rt_tar.st_data.vel_z > -3)
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
	dt.fun[0xf2].WTS = 1;
	
	if(angle - raspi.angle < 3 && angle - raspi.angle > -3)
	{
		rt_tar.st_data.yaw_dps = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}
