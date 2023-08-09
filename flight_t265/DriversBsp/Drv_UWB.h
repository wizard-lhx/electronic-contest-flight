#ifndef __DRV_UWB_H
#define __DRV_UWB_H

//==引用
#include "SysConfig.h"

//==定义/声明

typedef struct
{
	s16 pos_x;
	s16 pos_y;
} _uwb_st;

//==数据声明
extern _uwb_st uwb;
//==函数声明
//static
static void UWB_DataAnl(uint8_t *data_buf, uint8_t num);

//public
void UWB_GetOneByte(uint8_t data);
#endif
