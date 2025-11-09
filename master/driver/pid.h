#ifndef _pid_h_
#define _pid_h_

#include "stm32f10x.h"

typedef struct
{
    float err;
    float err_last;

    float expect;
    float feedback;

    float kp;
    float ki;
    float kd;

    float integral;
    float integral_max;

    float out;
    float out_max;
}Pid;


typedef struct
{
    //姿态外环（角度环）
    Pid pitAngle;
    Pid rolAngle;
    Pid yawAngle;

    //姿态内环（角速度环）
    Pid pitGyro;
    Pid rolGyro;
    Pid yawGyro;

    //竖直定高
    Pid acc_high;
    Pid vel_high;
    Pid pos_high;

    //定点
    Pid acc_fix_x;
    Pid vel_fix_x;
    Pid pos_fix_x;

    //定点
    Pid acc_fix_y;
    Pid vel_fix_y;
    Pid pos_fix_y;

}AllPid;

float PidController(Pid *controller);
void AllPidInit(void);
void ClearIntegral(Pid *controller);

#endif
