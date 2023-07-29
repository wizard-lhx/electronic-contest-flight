#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "My_Fun.h"

void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    //һ�����/��������
    //////////////////////////////////////////////////////////////////////
    //�þ�̬������¼һ�����/����ָ���Ѿ�ִ�С�
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step;
    //�ж���ң���źŲ�ִ��
    if (rc_in.fail_safe == 0)
    {
        //�жϵ�6ͨ������λ�� 1800<CH_6<2200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            //��û��ִ��
            if (one_key_takeoff_f == 0)
            {
                //����Ѿ�ִ��
                one_key_takeoff_f =
                    //ִ��һ�����
                    OneKey_Takeoff(100); //������λ�����ף� 0��Ĭ����λ�����õĸ߶ȡ�
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_takeoff_f = 0;
        }
        //
        //�жϵ�6ͨ������λ�� 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //��û��ִ��
            if (one_key_land_f == 0)
            {
                //����Ѿ�ִ��
                one_key_land_f =
                    //ִ��һ������
                    OneKey_Land();
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_land_f = 0;
        }
        //�жϵ�6ͨ������λ�� 1300<CH_6<1700
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {
            //��û��ִ��
            if (one_key_mission_f == 0)
            {
                //����Ѿ�ִ��
                one_key_mission_f = 1;
                //��ʼ����
                mission_step = 1;
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_mission_f = 0;
        }
        //
        if (one_key_mission_f == 1)
        {
			static u16 time_dly_cnt_ms;
			//
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
				//�л�����ģʽ
				mission_step += LX_Change_Mode(2);
			}
			break;
			case 2:
			{
				//����
				mission_step += FC_Unlock();
			}
			break;
			case 3:
			{
				//��2��
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
				mission_step += OneKey_Takeoff(100);
			}
			break;
			case 5:
			{
				//��5��
				if(time_dly_cnt_ms<5000)
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
				mission_step += Set_Position(1);
			}
			break;
			case 7:
			{
				//��2��
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
				mission_step += Set_Angle(90);
			}
			break;
			case 9:
			{
				//��2��
				if(time_dly_cnt_ms<2000)
				{
					time_dly_cnt_ms+=20;//ms
				}
				else
				{
					time_dly_cnt_ms = 0;
					mission_step = 12;
				}
			}
			break;
			case 10:
			{
				mission_step += Set_Position(3);
			}
			break;
			case 11:
			{
				//��2��
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
			case 12:
			{
				mission_step += OneKey_Land(); 
			}
			break;
			case 13:
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
