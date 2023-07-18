#include "Drv_RasPi.h"
#include "Drv_Uart.h"

_raspi_st raspi;
static uint8_t _datatemp[20];
u8 tx_buffer[20];

//RasPi_GetOneByte是初级数据解析函数，串口每接收到一字节树莓派数据，调用本函数一次，函数参数就是串口收到的数据
//当本函数多次被调用，最终接收到完整的一帧数据后，会自动调用数据解析函数RasPi_DataAnl
void RasPi_GetOneByte(uint8_t data)
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
	else if (rxstate == 2 && data < 25)
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
		RasPi_DataAnl(_datatemp, _data_len + 5); //
	}
	else
	{
		rxstate = 0;
	}
}
//RasPi_DataAnl为树莓派数据解析函数，可以通过本函数得到树莓派输出的各项数据

static void RasPi_DataAnl(uint8_t *data, uint8_t len)
{
	u8 check_sum1 = 0, check_sum2 = 0;
	if (*(data + 2) != (len - 5)) //判断数据长度是否正确
		return;
	for (u8 i = 0; i < len - 2; i++)
	{
		check_sum1 += *(data + i);
		check_sum2 += check_sum1;
	}
	if ((check_sum1 != *(data + len - 2)) || (check_sum2 != *(data + len - 1))) //判断sum校验
		return;
	//================================================================================

	if (*(data + 1) == 0X01) //树莓派的 T265 数据
	{
		raspi.x = *((s16 *)(data + 3));
		raspi.y = *((s16 *)(data + 5));
		raspi.z = *((s16 *)(data + 7));
		
		raspi.dx = *((s16 *)(data + 9));
		raspi.dy = *((s16 *)(data + 11));
		raspi.dz = *((s16 *)(data + 13));
		
		raspi.angle = *((s16 *)(data + 15)) * 0.01f;
	}
}

// 发送数据函数
void Send_Data_To_RasPi(u8* send_data, u8 data_len)
{
	tx_buffer[0] = 0xBB;
	tx_buffer[1] = 0x02;
	tx_buffer[2] = data_len;
	
	for(u8 i = 0; i < data_len; i++)
	{
		tx_buffer[3+i] = send_data[i];
	}
	
	u8 check_sum1 = 0, check_sum2 = 0;
	for(u8 j = 0; j < data_len; j++)
	{
		check_sum1 += send_data[j];
		check_sum2 += check_sum1;
	}
	
	tx_buffer[data_len+3] = check_sum1;
	tx_buffer[data_len+4] = check_sum2;
	// 实际发送
	LX_Send_To_RasPi(tx_buffer, data_len+5);
}
