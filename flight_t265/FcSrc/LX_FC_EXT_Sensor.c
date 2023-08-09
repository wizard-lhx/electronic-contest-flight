/*==========================================================================
 * 描述    ：凌霄飞控外置传感器处理
 * 更新时间：2020-02-06 
 * 作者		 ：匿名科创-Jyoun
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 项目合作：18084888982，18061373080
============================================================================
 * 匿名科创团队感谢大家的支持，欢迎大家进群互相交流、讨论、学习。
 * 若您觉得匿名有不好的地方，欢迎您拍砖提意见。
 * 若您觉得匿名好，请多多帮我们推荐，支持我们。
 * 匿名开源程序代码欢迎您的引用、延伸和拓展，不过在希望您在使用时能注明出处。
 * 君子坦荡荡，小人常戚戚，匿名坚决不会请水军、请喷子，也从未有过抹黑同行的行为。  
 * 开源不易，生活更不容易，希望大家互相尊重、互帮互助，共同进步。
 * 只有您的支持，匿名才能做得更好。  
===========================================================================*/
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "Drv_RasPi.h"
#include "ANO_DT_LX.h"
#include "Ano_Math.h"
#include "ANO_LX.h"

_fc_ext_sensor_st ext_sens;
init_qua_st init_qua;

//获取初始的飞机位姿
void Qua_Init(void)
{
	init_qua.init_qua_wx10000 = fc_att_qua.st_data.w_x10000;
	init_qua.init_qua_xx10000 = fc_att_qua.st_data.x_x10000;
	init_qua.init_qua_yx10000 = fc_att_qua.st_data.y_x10000;
	init_qua.init_qua_zx10000 = fc_att_qua.st_data.z_x10000;
}
//这里把T265数据打包成通用速度传感器数据
static inline void General_Velocity_Data_Handle()
{
	static u8 of_alt_update_cnt, of_update_cnt, t265_update_cnt; 
	static u8 dT_ms = 0;
	//每一毫秒dT_ms+1，用来判断是否长时间无数据
	if (dT_ms != 255)
	{
		dT_ms++;
	}
	//检查OF数据是否更新
	if (of_update_cnt != ano_of.of_update_cnt)
	{
		of_update_cnt = ano_of.of_update_cnt;
		//XY_VEL
		if (ano_of.of1_sta && ano_of.work_sta) //光流有效
		{
			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = ano_of.of1_dx;
			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = ano_of.of1_dy;
		}
		else //无效
		{
			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = 0x8000;
			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = 0x8000;
		}
	}
	// 想将 T265 速度数据替换掉光流数据，上位机上看速度数值方向已经正常，但实际测试飞机转过角度后就定不住点了
//	if(t265_update_cnt != raspi.t265_update_cnt)
//	{
//		t265_update_cnt = raspi.t265_update_cnt;
//		if(raspi.t265_status == 1) // t265 有效
//		{
////			float vector_t265_vx[2] = {raspi.dx*1.0f, 0.0f}; 
////			float vector_flight_vx[2];
////			rot_vec_2(vector_t265_vx, (float)my_sin(raspi.angle/100.0*ONE_PI/180.0), 
////				(float)my_cos(raspi.angle/100.0*ONE_PI/180.0), vector_flight_vx);
////	
////			float vector_t265_vy[2] = {0.0f, raspi.dy*1.0f};
////			float vector_flight_vy[2];
////			rot_vec_2(vector_t265_vy, (float)my_sin(raspi.angle/100.0*ONE_PI/180.0), 
////				(float)my_cos(raspi.angle/100.0*ONE_PI/180.0), vector_flight_vy);
////			
////			s16 flight_vel_x = (s16)((s32)(vector_flight_vx[0] + vector_flight_vy[0]));
////			s16 flight_vel_y = (s16)((s32)(vector_flight_vx[1] + vector_flight_vy[1]));

//			vector3d_t vector_t265 = {raspi.dx, raspi.dy, raspi.dz};
//			quaternion_t quaternion_init_flight = {init_qua.init_qua_wx10000/10000.0f, -init_qua.init_qua_xx10000/10000.0f,
//			-init_qua.init_qua_yx10000/10000.0f, -init_qua.init_qua_zx10000/10000.0f};
//			vector_t265 = rotate_vector(vector_t265, quaternion_init_flight);
//			quaternion_t quaternion_flight = {fc_att_qua.st_data.w_x10000/10000.0f, fc_att_qua.st_data.x_x10000/10000.0f,
//			fc_att_qua.st_data.y_x10000/10000.0f, fc_att_qua.st_data.z_x10000/10000.0f};
//			vector3d_t flight_vel = rotate_vector(vector_t265, quaternion_flight);
//			
//			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = (s16)flight_vel.x;
//			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = (s16)flight_vel.y;
//		}
//		else
//		{
//			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = 0x8000;
//			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = 0x8000;
//		}
//	}
//	//如果 t265 失效，检查OF数据是否更新
//	else if (of_update_cnt != ano_of.of_update_cnt && raspi.t265_status == 0)
//	{
//		of_update_cnt = ano_of.of_update_cnt;
//		//XY_VEL
//		if (ano_of.of1_sta && ano_of.work_sta) //光流有效
//		{
//			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = ano_of.of1_dx;
//			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = ano_of.of1_dy;
//		}
//		else //无效
//		{
//			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = 0x8000;
//			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = 0x8000;
//		}
//	}
	if (of_alt_update_cnt != ano_of.alt_update_cnt)
	{
		//
		of_alt_update_cnt = ano_of.alt_update_cnt;
		//不输入z轴速度，将z速度赋值为无效
		ext_sens.gen_vel.st_data.hca_velocity_cmps[2] = 0x8000;
		//触发发送
		dt.fun[0x33].WTS = 1;
		dt.fun[0xf1].WTS = 1;
		//reset
		dT_ms = 0;
	}
}

static inline void General_Distance_Data_Handle()
{
	static u8 of_alt_update_cnt;
	if (of_alt_update_cnt != ano_of.alt_update_cnt)
	{
		//
		of_alt_update_cnt = ano_of.alt_update_cnt;
		//
		ext_sens.gen_dis.st_data.direction = 0;
		ext_sens.gen_dis.st_data.angle_100 = 270;
		ext_sens.gen_dis.st_data.distance_cm = ano_of.of_alt_cm;
		//触发发送
		dt.fun[0x34].WTS = 1;
	}
}

//static inline void General_Position_Data_Handle()
//{
//	static u8 t265_update_cnt0;
//	if(t265_update_cnt0 != raspi.t265_update_cnt)
//	{
//		//
//		t265_update_cnt0 = raspi.t265_update_cnt;
//		//
//		ext_sens.gen_pos.st_data.ulhca_pos_cm[0] = raspi.x;
//		ext_sens.gen_pos.st_data.ulhca_pos_cm[1] = raspi.y;
//		ext_sens.gen_pos.st_data.ulhca_pos_cm[2] = 0x80000000;
//		//触发发送
//		dt.fun[0x32].WTS = 1;
//	}
//}

void LX_FC_EXT_Sensor_Task(float dT_s) //1ms
{
	//
	//General_Position_Data_Handle();
	//
	General_Velocity_Data_Handle();
	//
	General_Distance_Data_Handle();
}
