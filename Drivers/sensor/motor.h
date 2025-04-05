//
// Created by lsc on 25-3-26.
//

#ifndef MFLY_MOTOR_H
#define MFLY_MOTOR_H

#include "rc.h"
#include "stm32g4xx_hal.h"
#include "tim.h"
#include "quaternion.h"

typedef struct {
    float x;        // X轴加速度
    float y;        // Y轴加速度
    float z;        // Z轴加速度
} Accelerometer;

// 电机输出结构
typedef struct {
    int motor1;     // 右前电机 (顺时针旋转)
    int motor2;     // 右后电机 (逆时针旋转)
    int motor3;     // 左前电机 (逆时针旋转)
    int motor4;     // 左后电机 (顺时针旋转)
} MotorOutputs;

// PID控制器结构
typedef struct {
    float kP;                   // 比例系数
    float kI;                   // 积分系数
    float kD;                   // 微分系数
    float error;                // 当前误差
    float prevError;            // 上一次误差
    float integral;             // 积分项
    float derivative;           // 微分项
    float output;               // 输出值
    float outputLimit;          // 输出限制
} PIDController;

void updatePIDController(PIDController* pid, float setpoint, float measurement, float dt);

void controlLoop(float currentTime, EulerAngles* angles, RC_Input* rc);
#endif //MFLY_MOTOR_H
