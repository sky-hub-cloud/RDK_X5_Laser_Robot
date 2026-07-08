#include "main.h"

void PID_Init(PID *pid, float Kp, float Ki, float Kd, 
              int32_t I_max, int32_t I_min, int32_t Out_max, int32_t Out_min){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->I_max_limit = I_max;
    pid->I_min_limit = I_min;
    pid->Out_max_limit = Out_max;
    pid->Out_min_limit = Out_min;
    
    pid->Error[0] = 0;
    pid->Error[1] = 0;
    pid->Error[2] = 0;
    pid->Integral = 0;
    pid->OUT = 0;
    pid->Pout = pid->Iout = pid->Dout = 0;
}

/*位置式*/
void PID_Positional_out(PID *pid){
    
    // 1. 计算当前误差
    pid->Error[0] = pid->target - pid->feedback;
    
    // 2. 比例项
    pid->Pout = pid->Kp * pid->Error[0];
    
    // 3. 积分项（带积分限幅）
    pid->Integral += pid->Error[0];
    // 积分限幅
    if (pid->Integral > pid->I_max_limit) pid->Integral = pid->I_max_limit;
    if (pid->Integral < pid->I_min_limit) pid->Integral = pid->I_min_limit;

    pid->Iout = pid->Ki * pid->Integral;
    
    // 4. 微分项
    pid->Dout = pid->Kd * (pid->Error[0] - pid->Error[1]);
    
    // 5. 存储各分量（便于调试）
    pid->Pout = pid->Pout;
    pid->Iout = pid->Iout;
    pid->Dout = pid->Dout;
    
    // 6. 计算总输出
    float output = pid->Pout + pid->Iout + pid->Dout;
    
    // 7. 输出限幅
    if (output > pid->Out_max_limit) output = pid->Out_max_limit;
    if (output < pid->Out_min_limit) output = pid->Out_min_limit;
    
    pid->OUT = (int16_t)output;
    
    // 8. 更新误差历史
    pid->Error[2] = pid->Error[1];
    pid->Error[1] = pid->Error[0];
}

/*增量式*/
void PID_Incremental_out(PID *pid){

    float delta_out;
    
    // 1. 计算当前误差
    pid->Error[0] = pid->target - pid->feedback;
    
    // 2. 计算增量：ΔOUT = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2*e1+e2)
    float delta_P = pid->Kp * (pid->Error[0] - pid->Error[1]);
    float delta_I = pid->Ki * pid->Error[0];
    float delta_D = pid->Kd * (pid->Error[0] - 2 * pid->Error[1] + pid->Error[2]);
    
    delta_out = delta_P + delta_I + delta_D;
    
    // 3. 存储各分量
    pid->Pout = delta_P;
    pid->Iout = delta_I;
    pid->Dout = delta_D;
    
    // 4. 累加到输出（增量式核心）
    int16_t new_out = pid->OUT + (int16_t)delta_out;
    
    // 5. 输出限幅
    if (new_out > pid->Out_max_limit) new_out = pid->Out_max_limit;
    if (new_out < pid->Out_min_limit) new_out = pid->Out_min_limit;
    
    pid->OUT = new_out;
    
    // 6. 更新误差历史
    pid->Error[2] = pid->Error[1];
    pid->Error[1] = pid->Error[0];
}


uint16_t tim_cnt = 0;
PID Angel;
extern float Yaw;

PID X,Y;
extern uint16_t x_u16, y_u16;
uint16_t x_last, y_last;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM7){
        tim_cnt++;
        Angel.target = 0;
        Angel.feedback = Yaw;
        PID_Positional_out(&Angel);
			
				if(x_u16) x_last = x_u16; else x_last = 340;
				if(y_u16) y_last = y_u16; else y_last = 320;
				X.feedback = x_last;
				Y.feedback = y_u16;
				PID_Positional_out(&X);
				PID_Positional_out(&Y);
        //Set_4_Speed(-Angel.OUT,Angel.OUT,Angel.OUT,-Angel.OUT);
			


    }
}
