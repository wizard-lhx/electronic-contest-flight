#ifndef __MY_FUN_H__
#define __MY_FUN_H__

#include "SysConfig.h"

extern float flight_wz;
extern float vector_flight_vel[2];

u8 Set_Position(u8 position_num);
u8 Set_Angle(s16 angle);
void Land(void);
u8 Take_Off(s32);

#endif
