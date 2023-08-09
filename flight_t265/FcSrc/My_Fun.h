#ifndef __MY_FUN_H__
#define __MY_FUN_H__

#include "SysConfig.h"

extern float flight_wz;
extern float vector_flight_vel[2];
extern s16 s_x_set, s_y_set, s_angle_set;
extern s16 r_x_ref, r_y_ref, r_angle_ref, r_openmv_x_ref, r_openmv_y_ref;

u8 Set_Position(u8 position_num);
u8 Set_Angle(s16 angle);
void Land(void);
u8 Take_Off(s32);
u8 Set_OpenMV_Point(u8 is_up);

#endif
