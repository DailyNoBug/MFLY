//
// Created by lsc on 25-3-26.
//

#ifndef MFLY_PID_H
#define MFLY_PID_H
typedef struct {
    float Kp, Ki, Kd;      // PID参数
    float integral;        // 积分项
    float prev_error;      // 上一次误差
    float output;          // 输出值
    float integral_limit;  // 积分限幅
    float output_limit;    // 输出限幅
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float int_limit, float out_limit);
float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt);
#endif //MFLY_PID_H
