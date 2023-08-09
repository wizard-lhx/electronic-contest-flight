#include "Drv_UWB.h"   // UWB ���ڽ�������
#include "Drv_Uart.h"
#include "ANO_DT_LX.h"

_uwb_st uwb;
static uint8_t _datatemp[173];

//UWB_GetOneByte�ǳ������ݽ�������������ÿ���յ�һ�ֽ�UWB���ݣ����ñ�����һ�Σ������������Ǵ����յ�������
//����������α����ã����ս��յ�������һ֡���ݺ󣬻��Զ��������ݽ�������UWB_DataAnl
void UWB_GetOneByte(uint8_t data)
{
	static u8 frame_len = 0, _data_cnt = 0;
	static u8 rxstate = 0;

	if (rxstate == 0 && data == 0x55)
	{
		rxstate = 1;
		_datatemp[0] = data;
	}
	else if (rxstate == 1 && data == 0x04)
	{
		rxstate = 2;
		_datatemp[1] = data;
	}
	else if (rxstate == 2 && data == 0xac)
	{
		rxstate = 3;
		_datatemp[2] = data;
	}
	else if (rxstate == 3 && data == 0x00)
	{
		rxstate = 4;
		_datatemp[3] = data;
		frame_len = 172;
	}
	else if (rxstate == 4)
	{
		_datatemp[4 + _data_cnt] = data;
		_data_cnt++;
		if(_data_cnt >= frame_len - 4)
		{
			_data_cnt = 0;
			rxstate = 0;
			UWB_DataAnl(_datatemp, frame_len); //
		}
	}
	else
	{
		rxstate = 0;
	}
}
//UWB_DataAnlΪUWB���ݽ�������������ͨ���������õ�UWB����ĸ�������

static void UWB_DataAnl(uint8_t *data, uint8_t len)
{
	u8 check_sum = 0;
	if (*((u16*)(data + 2)) != 172) //�ж����ݳ����Ƿ���ȷ
		return;
	for (u8 i = 0; i < len - 1; i++)
	{
		check_sum += data[i];
	}
	if (check_sum != *(data + len - 1)) //�ж�sumУ��
		return;
	//================================================================================
    // 24 λ����ת 32 λ
	s32 temp = (int32_t)(data[13] << 8 | data[14] << 16 | data[15] << 24) / 256;
    // ��Ϊ��������������С�� +-32767 cm �����Խ� 32λ λ����ϢתΪ 16 λ	
	uwb.pos_x = (s16)(temp/10.0f);
	temp = (int32_t)(data[16] << 8 | data[17] << 16 | data[18] << 24) / 256; 
	uwb.pos_y = (s16)(temp/10.0f);
}
