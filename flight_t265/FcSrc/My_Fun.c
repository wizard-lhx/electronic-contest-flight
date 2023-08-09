#include "Drv_PID.h"
#include "ANO_LX.h"
//#include "Drv_RasPi.h"
#include "Drv_AnoOf.h"
#include "Ano_Math.h"
#include "Ano_DT_LX.h"
#include "Drv_OpenMV_Front.h"
#include "Drv_UWB.h"
// 自己封装的应用层飞行函数文件
//坐标
s16 position_x_set[12] = {50,  60,   360,  360,  120,  120,  360,  360,  120,  120,  350,  350};
s16 position_y_set[12] = {-60, -450, -450, -380, -380, -300, -300, -220, -220, -140, -140, -70};
s16 position_z_set[12] = {180, 180,  180,  180,  180,  180,  180,  180,  180,  180,  180,  180};
// 设置为全局变量，方便发送到上位机显示
float vector_flight_vel[2];
s16 s_x_set, s_y_set, s_angle_set;  // 坐标设定值
s16 r_x_ref, r_y_ref, r_angle_ref, r_openmv_x_ref, r_openmv_y_ref; // 坐标反馈值
u8 Set_Position(u8 position_num) // 传入航点编号
{
	float world_vel_x = PID_Cal(&pos_x_pid, position_x_set[position_num], uwb.pos_y);
	float world_vel_y = PID_Cal(&pos_y_pid, position_y_set[position_num], -uwb.pos_x);
	float world_vel_z = PID_Cal(&pos_z_pid, position_z_set[position_num], ano_of.of_alt_cm);

	float vector_world_vel[2] = {world_vel_x, world_vel_y};
	rot_vec_2(vector_world_vel, (float)my_sin(fc_att.st_data.yaw_x100/100.0*ONE_PI/180.0), 
		(float)my_cos(fc_att.st_data.yaw_x100/100.0*ONE_PI/180.0), vector_flight_vel);

	vector_flight_vel[0] = LIMIT(vector_flight_vel[0],-pos_x_pid.max_out,pos_x_pid.max_out);
	s16 flight_vel_x = (s16)vector_flight_vel[0];
	vector_flight_vel[1] = LIMIT(vector_flight_vel[1],-pos_y_pid.max_out,pos_y_pid.max_out);
	s16 flight_vel_y = (s16)vector_flight_vel[1];
	world_vel_z = LIMIT(world_vel_z,-pos_z_pid.max_out,pos_z_pid.max_out);
	s16 flight_vel_z = (s16)world_vel_z;

	rt_tar.st_data.vel_x = flight_vel_x;
	rt_tar.st_data.vel_y = flight_vel_y;
	rt_tar.st_data.vel_z = flight_vel_z;
	
	s_x_set = position_x_set[position_num];
	s_y_set = position_y_set[position_num];
	r_x_ref = uwb.pos_y;
	r_y_ref = -uwb.pos_x;
	dt.fun[0xf2].WTS = 1;// 方便pid调参用

	if(position_x_set[position_num] - uwb.pos_y < 10 && position_x_set[position_num] - uwb.pos_y > -10 && 
		position_y_set[position_num] + uwb.pos_x < 10 && position_y_set[position_num] + uwb.pos_x > -10 && 
		position_z_set[position_num] - (s16)ano_of.of_alt_cm < 3 && position_z_set[position_num] - (s16)ano_of.of_alt_cm > -3)
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
	flight_wz = PID_Cal(&angle_pid, angle, fc_att.st_data.yaw_x100/100.0f);
	
	flight_wz = LIMIT(flight_wz,-angle_pid.max_out,angle_pid.max_out);
	
	rt_tar.st_data.yaw_dps = -((s16)flight_wz);

	s_angle_set = angle;
	r_angle_ref = fc_att.st_data.yaw_x100/100.0f;
	dt.fun[0xf2].WTS = 1;// 方便pid调参用
	
	if(angle - fc_att.st_data.yaw_x100/100.0 < 3 && angle - fc_att.st_data.yaw_x100/100.0 > -3)
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

u8 Set_OpenMV_Point(u8 is_up) // 传入OpenMV识别时飞机是降落放灭火包还是保持
{
	float world_vel_x = PID_Cal(&pos_x_pid, -openmv_front.dy, 0.00f);
	float world_vel_y = PID_Cal(&pos_y_pid, -openmv_front.dx, 0.00f);
	float world_vel_z = 0;
	if(is_up == 1)
	{
		world_vel_z = PID_Cal(&pos_z_pid, 180.0f, ano_of.of_alt_cm);
	}
	else
	{
		world_vel_z = PID_Cal(&pos_z_pid, 100.0f, ano_of.of_alt_cm);
	}
	
	float vector_world_vel[2] = {world_vel_x, world_vel_y};
	rot_vec_2(vector_world_vel, (float)my_sin(fc_att.st_data.yaw_x100/100.0*ONE_PI/180.0), 
		(float)my_cos(fc_att.st_data.yaw_x100/100.0*ONE_PI/180.0), vector_flight_vel);
	
	vector_flight_vel[0] = LIMIT(vector_flight_vel[0],-pos_x_pid.max_out,pos_x_pid.max_out);
	s16 flight_vel_x = (s16)vector_flight_vel[0];
	vector_flight_vel[1] = LIMIT(vector_flight_vel[1],-pos_y_pid.max_out,pos_y_pid.max_out);
	s16 flight_vel_y = (s16)vector_flight_vel[1];
	world_vel_z = LIMIT(world_vel_z,-pos_z_pid.max_out,pos_z_pid.max_out);
	s16 flight_vel_z = (s16)world_vel_z;
	
	rt_tar.st_data.vel_x = flight_vel_x;
	rt_tar.st_data.vel_y = flight_vel_y;
	rt_tar.st_data.vel_z = flight_vel_z;
	
	r_openmv_x_ref = -openmv_front.dy;
	r_openmv_y_ref = -openmv_front.dx;
	dt.fun[0xf2].WTS = 1;// 方便pid调参用
	
	if(openmv_front.dx < 3 && openmv_front.dx > -3 && openmv_front.dy < 3 && openmv_front.dy > -3 &&
		openmv_front.state == 1)
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
