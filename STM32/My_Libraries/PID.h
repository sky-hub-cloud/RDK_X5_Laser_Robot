#ifndef __PID_H
#define __PID_H

typedef struct {

    float target,feedback;

    int32_t Error[3];

    float Kp,Ki,Kd;
    int32_t Pout, Iout, Dout;

    int32_t I_max_limit;
    int32_t I_min_limit;
    int32_t Integral; 

    int32_t Out_max_limit;
    int32_t Out_min_limit;

    int32_t OUT;

} PID;

void PID_Init(PID *pid, float Kp, float Ki, float Kd, 
            int32_t I_max, int32_t I_min, int32_t Out_max, int32_t Out_min);
void PID_Positional_out(PID *pid);
void PID_Incremental_out(PID *pid);


#endif
