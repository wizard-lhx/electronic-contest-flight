#include "Drv_ANO_DT.h" // 数传串口驱动函数，主要与小车进行通信。
#include "Drv_Uart.h"

_ano_dt_st ano_dt;
u8 car_cmd = 0;
static uint8_t _datatemp[10];
static u8 tx_buffer[13];

//ANO_DT_GetOneByte是初级数据解析函数，串口每接收到一字节小车数据，调用本函数一次，函数参数就是串口收到的数据
//当本函数多次被调用，最终接收到完整的一帧数据后，会自动调用数据解析函数ANO_DT_DataAnl
void ANO_DT_GetOneByte(uint8_t data)
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
		ANO_DT_DataAnl(_datatemp, _data_len + 5); //
	}
	else
	{
		rxstate = 0;
	}
}
//ANO_DT_DataAnl为数传数据解析函数，可以通过本函数得到数传输出的各项数据

static void ANO_DT_DataAnl(uint8_t *data, uint8_t len)
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

	if (*(data + 1) == 0X02) //ano_dt 的数据
	{
		car_cmd = *(data + 3);
	}
}

// 发送数据函数
void Send_Data_To_ANO_DT(u8 data_len)
{
	tx_buffer[0] = 0xBB;
	tx_buffer[1] = 0x02;
	tx_buffer[2] = data_len;
	// 因为发送到小车数传的数据是固定不变的，所以将内容写死
	tx_buffer[3] = BYTE0(ano_dt.x);
	tx_buffer[4] = BYTE1(ano_dt.x);
	tx_buffer[5] = BYTE0(ano_dt.y);
	tx_buffer[6] = BYTE1(ano_dt.y);
	tx_buffer[7] = BYTE0(ano_dt.voyage);
	tx_buffer[8] = BYTE1(ano_dt.voyage);
	tx_buffer[9] = ano_dt.state;
	
	u8 check_sum1 = 0, check_sum2 = 0;
	for(u8 j = 0; j < data_len + 3; j++)
	{
		check_sum1 += tx_buffer[j];
		check_sum2 += check_sum1;
	}
	
	tx_buffer[data_len+3] = check_sum1;
	tx_buffer[data_len+4] = check_sum2;
	// 实际发送
	LX_Send_To_Car(tx_buffer, data_len+5);
}
