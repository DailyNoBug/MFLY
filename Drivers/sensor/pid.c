//
// Created by lsc on 25-3-26.
//

#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float int_limit, float out_limit) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
    pid->integral_limit = int_limit;
    pid->output_limit = out_limit;
}

float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    // 比例项
    float P = pid->Kp * error;

    // 积分项（抗饱和）
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float I = pid->Ki * pid->integral;

    // 微分项（考虑测量值变化，而非误差变化）
    float D = pid->Kd * ( (error - pid->prev_error) / dt );
    pid->prev_error = error;

    // 计算总输出
    pid->output = P + I + D;

    // 输出限幅
    if (pid->output > pid->output_limit) pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;

    return pid->output;
}