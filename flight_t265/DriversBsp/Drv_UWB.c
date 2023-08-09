#include "Drv_UWB.h"   // UWB 串口接收驱动
#include "Drv_Uart.h"
#include "ANO_DT_LX.h"

_uwb_st uwb;
static uint8_t _datatemp[173];

//UWB_GetOneByte是初级数据解析函数，串口每接收到一字节UWB数据，调用本函数一次，函数参数就是串口收到的数据
//当本函数多次被调用，最终接收到完整的一帧数据后，会自动调用数据解析函数UWB_DataAnl
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
//UWB_DataAnl为UWB数据解析函数，可以通过本函数得到UWB输出的各项数据

static void UWB_DataAnl(uint8_t *data, uint8_t len)
{
	u8 check_sum = 0;
	if (*((u16*)(data + 2)) != 172) //判断数据长度是否正确
		return;
	for (u8 i = 0; i < len - 1; i++)
	{
		check_sum += data[i];
	}
	if (check_sum != *(data + len - 1)) //判断sum校验
		return;
	//================================================================================
    // 24 位数据转 32 位
	s32 temp = (int32_t)(data[13] << 8 | data[14] << 16 | data[15] << 24) / 256;
    // 因为比赛场地内坐标小于 +-32767 cm ，所以将 32位 位置信息转为 16 位	
	uwb.pos_x = (s16)(temp/10.0f);
	temp = (int32_t)(data[16] << 8 | data[17] << 16 | data[18] << 24) / 256; 
	uwb.pos_y = (s16)(temp/10.0f);
}
