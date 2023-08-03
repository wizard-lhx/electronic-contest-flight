#ifndef __DRV_OPENMV_FRONT_H
#define __DRV_OPENMV_FRONT_H

//==引用
#include "SysConfig.h"

//==定义/声明

typedef struct
{
	s16 dx;
	s16 dy;
} _openmv_front_st;

//==数据声明
extern _openmv_front_st openmv_front;
//==函数声明
//static
static void OpenMV_Front_DataAnl(uint8_t *data_buf, uint8_t num);

//public
void OpenMV_Front_GetOneByte(uint8_t data);
void Send_Data_To_OpenMV_Front(u8* send_data, u8 data_len);
#endif
