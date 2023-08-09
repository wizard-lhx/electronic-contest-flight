#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "My_Fun.h"
//#include "Drv_RasPi.h"
#include "Drv_OpenMV_Front.h"
#include "Drv_ANO_DT.h"
#include "Drv_UWB.h"
#include "ANO_DT_LX.h"

u8 mission_step;//static u8 mission_step;
// 发送到OpneMV的指令队列
u8 openmv_front_cmd[5] = {0x02, 0x03, 0x04, 0x05};

void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    //一键起飞/降落例程
    //////////////////////////////////////////////////////////////////////
    //用静态变量记录一键起飞/降落指令已经执行。
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    //判断有遥控信号才执行
    if (rc_in.fail_safe == 0)
    {
        //判断第6通道拨杆位置 1800<CH_6<2200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            //还没有执行
            if (one_key_takeoff_f == 0)
            {
                //标记已经执行
                one_key_takeoff_f =
                    //执行一键起飞
                    OneKey_Takeoff(100); //参数单位：厘米； 0：默认上位机设置的高度。
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_takeoff_f = 0;
        }
        //
        //判断第6通道拨杆位置 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //还没有执行
            if (one_key_land_f == 0)
            {
                //标记已经执行
                one_key_land_f =
                    //执行一键降落
                    OneKey_Land();
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_land_f = 0;
        }
        //判断第6通道拨杆位置 1300<CH_6<1700
        if ((rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700) ||
			car_cmd == 1) //记得改回来
        {
            //还没有执行
            if (one_key_mission_f == 0)
            {
                //标记已经执行
                one_key_mission_f = 1;
                //开始流程
                mission_step = 1;
				//mission_step = 29; // 测试通信用
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_mission_f = 0;
        }
        //
        if (one_key_mission_f == 1)
        {
			ano_dt.x = uwb.pos_y;
			ano_dt.y = -uwb.pos_x;
			static s16 voyage_base; // 航程基础累计
			s16 voyage_add;         // 航程增加累计
			static u16 time_dly_cnt_ms;
			//
			static u16 my_cnt_ms1;   //计时发现时长
			static u16 my_cnt_ms2;   //计时没发现时长
			static u8 openmv_mission_step;
			static u8 openmv_mission_start_flag;
			static u8 openmv_mission_end_flag;
			if(rc_in.rc_ch.st_data.ch_[ch_7_aux3] > 1800 && rc_in.rc_ch.st_data.ch_[ch_7_aux3] < 2200 && 
				openmv_front.state == 1 && openmv_mission_end_flag == 0 && openmv_mission_start_flag == 1)
			{
				my_cnt_ms2 = 0;
				//openmv_mission_start_flag = 1;
				switch(openmv_mission_step)
				{
				case 0:
				{
					openmv_mission_step += Set_OpenMV_Point(1);
					if(openmv_mission_step == 1)
					{   
						// LED
						for(u8 i = 0;i < 2;i++)	
							Send_Data_To_OpenMV_Front(&openmv_front_cmd[1], 1);
					}
				}
				break;
				case 1:
				{
					openmv_mission_step += Set_OpenMV_Point(0);
				}
				break;
				case 2:
				{
					//等2秒
					if(time_dly_cnt_ms<2000)
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
						if(openmv_mission_step == 3)
						{
							// 舵机
							ano_dt.state = 1;
							for(u8 i = 0;i < 2;i++)
							{
								Send_Data_To_ANO_DT(7);//到达火情点立刻发坐标
								Send_Data_To_OpenMV_Front(&openmv_front_cmd[2], 1);
							}
						}
					}
				}
				break;
				case 3:
				{
					ano_dt.state = 0;
					openmv_mission_step += Set_OpenMV_Point(1);
					if(openmv_mission_step == 4)
					{   
						openmv_mission_start_flag = 0;
						openmv_mission_end_flag = 1;
						// 关LED
						for(u8 i = 0;i < 2;i++)
							Send_Data_To_OpenMV_Front(&openmv_front_cmd[3], 1);
					}
				}
				break;
				default:
				break;
				}				
			}
			else if(openmv_mission_start_flag != 1 && openmv_front.state != 1)
			{
				my_cnt_ms1 = 0;
				switch(mission_step)
				{
				case 0:
				{
					//reset
					time_dly_cnt_ms = 0;
				}
				break;
				case 1:
				{
					//切换定点模式
					mission_step += LX_Change_Mode(2);
					//mission_step = 29; // 测试通信用
				}
				break;
				case 2:
				{
					//解锁
					mission_step += FC_Unlock();
					//Send_Data_To_OpenMV_Front(openmv_front_cmd, 1);
				}
				break;
				case 3:
				{
					//等2秒
					if(time_dly_cnt_ms<2000)
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 4:
				{
					mission_step += Take_Off(180);
				}
				break;
				case 5:
				{
					mission_step += Set_Angle(0);
				}
				break;
				case 6:
				{
					mission_step += Set_Position(0);
					if(mission_step == 7)
						mission_step += 1;
				}
				break;
				case 7:
				{
					
				}
				break;
				case 8:
				{
					mission_step += Set_Position(1);
					voyage_add = uwb.pos_x;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 9)
					{
						voyage_base += voyage_add;
						if(rc_in.rc_ch.st_data.ch_[ch_7_aux3] > 1800 && rc_in.rc_ch.st_data.ch_[ch_7_aux3] < 2200)
							for(u8 i = 0;i < 2;i++)
								Send_Data_To_OpenMV_Front(&openmv_front_cmd[0], 1);
					}
				}
				break;
				case 9:
				{
					mission_step += Set_Angle(0);
				}
				break;
				case 10:
				{
					mission_step += Set_Position(2);
					voyage_add = uwb.pos_y;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 11)
						voyage_base += voyage_add;
				}
				break;
				case 11:
				{
					mission_step += Set_Position(3);
					voyage_add = -uwb.pos_x + 400;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 12)
						voyage_base += voyage_add;
				}
				break;
				case 12:
				{
					mission_step += Set_Position(4);
					voyage_add = 320 - uwb.pos_y;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 13)
						voyage_base += voyage_add;
				}
				break;
				case 13:
				{
					mission_step += Set_Angle(0);
				}
				break;
				case 14:
				{
					mission_step += Set_Position(5);
					voyage_add = -uwb.pos_x + 320;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 15)
						voyage_base += voyage_add;
				}
				break;
				case 15:
				{
					mission_step += Set_Position(6);
					voyage_add = uwb.pos_y - 80;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 16)
						voyage_base += voyage_add;
				}
				break;
				case 16:
				{
					mission_step += Set_Angle(0);
				}
				break;
				case 17:
				{
					mission_step += Set_Position(7);
					voyage_add = -uwb.pos_x + 240;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 18)
						voyage_base += voyage_add;
				}
				break;
				case 18:
				{
					mission_step += Set_Position(8);
					voyage_add = 320 - uwb.pos_y;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 19)
						voyage_base += voyage_add;
				}
				break;
				case 19:
				{
					mission_step += Set_Angle(0);
				}
				break;
				case 20:
				{
					mission_step += Set_Position(9);
					voyage_add = -uwb.pos_x + 160;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 21)
						voyage_base += voyage_add;
				}
				break;
				case 21:
				{
					mission_step += Set_Position(10);
					voyage_add = uwb.pos_y - 80;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 22)
						voyage_base += voyage_add;
				}
				break;
				case 22:
				{
					mission_step += Set_Angle(0);
				}
				break;
				case 23:
				{
					mission_step += Set_Position(11);
					voyage_add = -uwb.pos_x + 80;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 24)
						voyage_base += voyage_add;
				}
				break;
				case 24:
				{
					mission_step += Set_Position(0);
					voyage_add = 320 - uwb.pos_y;
					ano_dt.voyage = voyage_base + voyage_add;
					if(mission_step == 25)
						voyage_base += voyage_add;
				}
				break;
				case 25:
				{
					//10秒内降落
					if(time_dly_cnt_ms<10000)
					{
						time_dly_cnt_ms+=20;//ms
						Land();
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 26:
				{
					rt_tar.st_data.vel_z = 0;
					car_cmd = 0;
				}
				break;
				case 27:
				{
					
				}
				break;
				case 28:
				{
					
				}
				break;
				case 29: //之后都是测试用
				{
					for(u8 i = 0;i < 2;i++)
						Send_Data_To_OpenMV_Front(&openmv_front_cmd[0],1);
					mission_step++;
				}
				break;
				case 30:
				{
					//等10秒
					if(time_dly_cnt_ms<10000)
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 31:
				{
					for(u8 i = 0;i < 2;i++)
						Send_Data_To_OpenMV_Front(&openmv_front_cmd[1],1);
						ano_dt.state = 1;
						Send_Data_To_ANO_DT(7);
					mission_step++;
				}
				break;
				case 32:
				{
					//等10秒
					if(time_dly_cnt_ms<10000)
					{
						ano_dt.state = 0;
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 33:
				{
					for(u8 i = 0;i < 2;i++)
						Send_Data_To_OpenMV_Front(&openmv_front_cmd[2],1);
					mission_step++;
				}
				break;
				case 34:
				{
					//等10秒
					if(time_dly_cnt_ms<10000)
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 35:
				{
					for(u8 i = 0;i < 2;i++)
						Send_Data_To_OpenMV_Front(&openmv_front_cmd[3],1);
					mission_step++;
				}
				break;
				case 36:
				{
					
				}
				break;
				default:break;
				}
			}
			else if(openmv_front.state == 1 && openmv_mission_start_flag != 1 && openmv_mission_end_flag == 0)
			{
				//等200毫秒
				if(my_cnt_ms1<200)
				{
					my_cnt_ms1+=20;//ms
					rt_tar.st_data.vel_x = 0;
					rt_tar.st_data.vel_y = 0;
					rt_tar.st_data.vel_z = 0;
					dt.fun[0xf2].WTS = 1;
				}
				else
				{
					my_cnt_ms1 = 0;
					openmv_mission_start_flag = 1;
				}
			}
			else if(openmv_mission_start_flag == 1 && openmv_front.state != 1 && openmv_mission_end_flag == 0)
			{
				//等2秒
				if(my_cnt_ms2<2000)
				{
					my_cnt_ms2+=20;//ms
					rt_tar.st_data.vel_x = 0;
					rt_tar.st_data.vel_y = 0;
					rt_tar.st_data.vel_z = 0;
					dt.fun[0xf2].WTS = 1;
				}
				else
				{
					my_cnt_ms2 = 0;
					openmv_mission_start_flag = 0;
				}
			}
        }
        else
        {
            mission_step = 0;
        }
    }
    ////////////////////////////////////////////////////////////////////////
}
