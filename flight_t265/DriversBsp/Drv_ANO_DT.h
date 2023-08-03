#ifndef __DRV_ANO_DT_H
#define __DRV_ANO_DT_H

//==����
#include "SysConfig.h"

//==����/����

typedef struct
{
	s16 x;
	s16 y;
	s16 voyage;
	u8 state;
} _ano_dt_st;

//==��������
extern _ano_dt_st ano_dt;
extern u8 car_cmd;
//==��������
//static
static void ANO_DT_DataAnl(uint8_t *data_buf, uint8_t num);

//public
void ANO_DT_GetOneByte(uint8_t data);
void Send_Data_To_ANO_DT(u8 data_len);
#endif
