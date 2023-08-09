#include "Drv_OpenMV_Front.h" // OpenMV ���ڽ��շ��ͺ��������Խ���OpenMV������λ��ƫ��Լ�����������OpenMVִ��
#include "Drv_Uart.h"
#include "ANO_DT_LX.h"

_openmv_front_st openmv_front;
static uint8_t _datatemp[30];
static u8 tx_buffer[20];

//OpenMV_Front_GetOneByte�ǳ������ݽ�������������ÿ���յ�һ�ֽ�OpenMV���ݣ����ñ�����һ�Σ������������Ǵ����յ�������
//����������α����ã����ս��յ�������һ֡���ݺ󣬻��Զ��������ݽ�������OpenMV_Front_DataAnl
void OpenMV_Front_GetOneByte(uint8_t data)
{
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 rxstate = 0;

	if (rxstate == 0 && data == 0xBB)
	{
		rxstate = 1;
		_datatemp[0] = data;
	}
	else if (rxstate == 1)
	{
		rxstate = 2;
		_datatemp[1] = data;
	}
	else if (rxstate == 2 && data < 6)
	{
		rxstate = 3;
		_datatemp[2] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if (rxstate == 3 && _data_len > 0)
	{
		_datatemp[3 + _data_cnt] = data;
		_data_cnt += 1;
		if(_data_cnt >= _data_len)
		{
			_data_cnt = 0;
			rxstate = 4;
		}
	}
	else if (rxstate == 4)
	{
		rxstate = 5;
		_datatemp[3 + _data_len] = data;
	}
	else if (rxstate == 5)
	{
		rxstate = 0;
		_datatemp[4 + _data_len] = data;
		OpenMV_Front_DataAnl(_datatemp, _data_len + 5); //
	}
	else
	{
		rxstate = 0;
	}
}
//OpenMV_Front_DataAnlΪOpenMV���ݽ�������������ͨ���������õ�OpenMV����ĸ�������

static void OpenMV_Front_DataAnl(uint8_t *data, uint8_t len)
{
	u8 check_sum1 = 0, check_sum2 = 0;
	if (*(data + 2) != (len - 5)) //�ж����ݳ����Ƿ���ȷ
		return;
	for (u8 i = 0; i < len - 2; i++)
	{
		check_sum1 += *(data + i);
		check_sum2 += check_sum1;
	}
	if ((check_sum1 != *(data + len - 2)) || (check_sum2 != *(data + len - 1))) //�ж�sumУ��
		return;
	//================================================================================

	if (*(data + 1) == 0X03) //openmv ������
	{
		openmv_front.dx = *((s16 *)(data + 3));
		openmv_front.dy = *((s16 *)(data + 5));
		openmv_front.state = *(data + 7);
		//dt.fun[0xf2].WTS = 1;
	}
}

// �������ݺ���
void Send_Data_To_OpenMV_Front(u8* send_data, u8 data_len)
{
	tx_buffer[0] = 0xBB;
	tx_buffer[1] = 0x03;
	tx_buffer[2] = data_len;
	
	for(u8 i = 0; i < data_len; i++)
	{
		tx_buffer[3+i] = send_data[i];
	}
	
	u8 check_sum1 = 0, check_sum2 = 0;
	for(u8 j = 0; j < data_len + 3; j++)
	{
		check_sum1 += tx_buffer[j];
		check_sum2 += check_sum1;
	}
	
	tx_buffer[data_len+3] = check_sum1;
	tx_buffer[data_len+4] = check_sum2;
	// ʵ�ʷ���
	LX_Send_To_OpenMV_Front(tx_buffer, data_len+5);
}
