#ifndef __DRV_ANO_DT_H
#define __DRV_ANO_DT_H

//==引用
#include "SysConfig.h"

//==定义/声明

typedef struct
{
	s16 x;
	s16 y;
	s16 voyage;
	u8 state;
} _ano_dt_st;

//==数据声明
extern _ano_dt_st ano_dt;
extern u8 car_cmd;
//==函数声明
//static
static void ANO_DT_DataAnl(uint8_t *data_buf, uint8_t num);

//public
void ANO_DT_GetOneByte(uint8_t data);
void Send_Data_To_ANO_DT(u8 data_len);
#endif
