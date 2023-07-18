#ifndef __DRV_RASPI_H
#define __DRV_RASPI_H

//==引用
#include "SysConfig.h"

//==定义/声明

typedef struct
{
	s16 x;
	s16 y;
	s16 z;
	
	s16 dx;
	s16 dy;
	s16 dz;
	
	float angle;
} _raspi_st;

//==数据声明
extern _raspi_st raspi;
//==函数声明
//static
static void RasPi_DataAnl(uint8_t *data_buf, uint8_t num);

//public
void RasPi_GetOneByte(uint8_t data);
void Send_Data_To_RasPi(u8* send_data, u8 data_len);
#endif
