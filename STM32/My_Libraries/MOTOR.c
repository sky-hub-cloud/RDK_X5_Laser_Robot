#include "main.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void Set_Speed(TIM_HandleTypeDef *htim, int32_t Speed){

    if(htim == &htim1){
        if(Speed >= 0){
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, Speed);
        }
        else{
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, -Speed);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0);
        }
    }
    else if(htim == &htim2){
        if(Speed >= 0){
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, Speed);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0);
        }
        else{
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, -Speed);
        }

    }
    else if(htim == &htim3){
        if(Speed >= 0){
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, Speed);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0);
        }
        else{
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, -Speed);
        }
    }
    else if(htim == &htim4){
        if(Speed >= 0){
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, Speed);
        }
        else{
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, -Speed);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0);
        }
    }
}

void Set_4_Speed(int32_t Speed_1,int32_t Speed_2,int32_t Speed_3,int32_t Speed_4){

    Set_Speed(&htim3,Speed_1);//ÂÖÌ¥1
    Set_Speed(&htim2,Speed_2);//ÂÖÌ¥2
    Set_Speed(&htim4,Speed_3);//ÂÖÌ¥3
    Set_Speed(&htim1,Speed_4);//ÂÖÌ¥4
}
