#ifndef __MOTOR_H
#define __MOTOR_H

void Set_Speed(TIM_HandleTypeDef *htim, int32_t Speed);
void Set_4_Speed(int32_t Speed_1,int32_t Speed_2,int32_t Speed_3,int32_t Speed_4);


#endif
