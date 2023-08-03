#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "My_Fun.h"
#include "Drv_RasPi.h"
#include "Drv_OpenMV_Front.h"
#include "Drv_ANO_DT.h"

u8 mission_step;//static u8 mission_step;

u8 openmv_front_cmd[5] = {0x02};

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
			ano_dt.x = raspi.x;
			ano_dt.y = raspi.y;
			static s16 voyage_base; // 航程基础累计
			s16 voyage_add;         // 航程增加累计
			static u16 time_dly_cnt_ms;
			//
			//mission_step = 27; // 测试小车通信用
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
			case 6:
			{
				mission_step += Set_Angle(0);
			}
			break;
			case 7:
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
			case 8:
			{
				mission_step += Set_Position(1);
				voyage_add = -raspi.y;
				ano_dt.voyage = voyage_base + voyage_add;
				if(mission_step == 9)
					voyage_base += voyage_add;
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
				voyage_add = raspi.x;
				ano_dt.voyage = voyage_base + voyage_add;
				if(mission_step == 11)
					voyage_base += voyage_add;
			}
			break;
			case 11:
			{
				mission_step += Set_Position(3);
				voyage_add = raspi.y + 400;
				ano_dt.voyage = voyage_base + voyage_add;
				if(mission_step == 12)
					voyage_base += voyage_add;
			}
			break;
			case 12:
			{
				mission_step += Set_Position(4);
				voyage_add = 320 - raspi.x;
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
				voyage_add = raspi.y + 320;
				ano_dt.voyage = voyage_base + voyage_add;
				if(mission_step == 15)
					voyage_base += voyage_add;
			}
			break;
			case 15:
			{
				mission_step += Set_Position(6);
				voyage_add = raspi.x - 80;
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
				voyage_add = raspi.y + 240;
				ano_dt.voyage = voyage_base + voyage_add;
				if(mission_step == 18)
					voyage_base += voyage_add;
			}
			break;
			case 18:
			{
				mission_step += Set_Position(8);
				voyage_add = 320 - raspi.x;
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
				voyage_add = raspi.y + 160;
				ano_dt.voyage = voyage_base + voyage_add;
				if(mission_step == 21)
					voyage_base += voyage_add;
			}
			break;
			case 21:
			{
				mission_step += Set_Position(10);
				voyage_add = raspi.x - 80;
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
				voyage_add = raspi.y + 80;
				ano_dt.voyage = voyage_base + voyage_add;
				if(mission_step == 24)
					voyage_base += voyage_add;
			}
			break;
			case 24:
			{
				mission_step += Set_Position(0);
				voyage_add = 320 - raspi.x;
				ano_dt.voyage = voyage_base + voyage_add;
				if(mission_step == 25)
					voyage_base += voyage_add;
			}
			break;
			case 25:
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
			case 26:
			{
				Land();
			}
			break;
			case 27:
			{
				
			}
			break;
			default:break;
			}
        }
        else
        {
            mission_step = 0;
        }
    }
    ////////////////////////////////////////////////////////////////////////
}
