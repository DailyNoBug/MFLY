//
// Created by lsc on 25-3-26.
//

#include "motor.h"

// 姿态控制结构 (外环)
PIDController rollAnglePID = {0.5, 0.4, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 300.0};  // 横滚角度PID
PIDController pitchAnglePID = {0.5, 0.4, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 400.0}; // 俯仰角度PID
PIDController yawAnglePID = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 200.0};   // 航向角度PID

// 角速度控制结构 (内环)
PIDController rollRatePID = {0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 300.0};   // 横滚角速度PID
PIDController pitchRatePID = {0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 300.0};  // 俯仰角速度PID
PIDController yawRatePID = {0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 200.0};     // 航向角速度PID

// 保存上一次姿态数据用于计算角速度
EulerAngles prevAngles = {0.0, 0.0, 0.0};
float prevTime = 0.0;
float last_yawSetPoint = 0.0;

// 限制值在指定范围内
float constrain(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

// 更新PID控制器
void updatePIDController(PIDController* pid, float setpoint, float measurement, float dt) {
    // 计算误差
    pid->error = setpoint - measurement;

    // 积分项
    pid->integral += pid->error * dt;

    // 限制积分项防止积分饱和
    pid->integral = constrain(pid->integral, -pid->outputLimit / pid->kI, pid->outputLimit / pid->kI);

    // 微分项
    pid->derivative = (pid->error - pid->prevError) / dt;

    // 计算PID输出
    pid->output = (pid->kP * pid->error) + (pid->kI * pid->integral) + (pid->kD * pid->derivative);

    // 限制输出范围
    pid->output = constrain(pid->output, -pid->outputLimit, pid->outputLimit);

    // 保存当前误差为上一次误差
    pid->prevError = pid->error;
}

// 计算角速度
float calculateAngularRate(float currentAngle, float prevAngle, float dt) {
    // 处理穿越180度/-180度边界的情况
    float diff = currentAngle - prevAngle;
    if (diff > 180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;

    return diff / dt;
}

// 计算电机输出
void calculateMotorOutputs(float throttle, float rollOutput, float pitchOutput, float yawOutput, MotorOutputs* outputs) {
    // 油门基础值 (0-1000)
    float baseThrottle = throttle * 1000.0 + 1000.0;

    // X型四轴布局的混控算法
    // 电机1：右前 (顺时针旋转)
    // 电机2：右后 (逆时针旋转)
    // 电机3：左前 (逆时针旋转)
    // 电机4：左后 (顺时针旋转)

    /*
        电机布局：
        3   1
         \ /
         / \
        4   2
    */

    // 计算各个电机的输出值
    outputs->motor1 = (int)(baseThrottle - rollOutput - pitchOutput - yawOutput); // 右前
    outputs->motor2 = (int)(baseThrottle - rollOutput + pitchOutput + yawOutput); // 右后
    outputs->motor3 = (int)(baseThrottle + rollOutput - pitchOutput + yawOutput); // 左前
    outputs->motor4 = (int)(baseThrottle + rollOutput + pitchOutput - yawOutput); // 左后

    // 确保电机输出在有效范围内 (假设为0-1000)
    outputs->motor1 = (int)constrain(outputs->motor1, 1000, 2000);
    outputs->motor2 = (int)constrain(outputs->motor2, 1000, 2000);
    outputs->motor3 = (int)constrain(outputs->motor3, 1000, 2000);
    outputs->motor4 = (int)constrain(outputs->motor4, 1000, 2000);
}

// 设置PWM输出
void motor_ctrl(int motor1,int motor2,int motor3,int motor4)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motor4);
}

// 主控制循环
void controlLoop(float currentTime, EulerAngles* angles, RC_Input* rc) {
    // 计算时间增量
    float dt = currentTime - prevTime;
    if (dt <= 0.0) dt = 0.01; // 防止除以零
    prevTime = currentTime;

    // 计算角速度 (可以用陀螺仪数据代替，如果有的话)
    float rollRate = calculateAngularRate(angles->roll, prevAngles.roll, dt);
    float pitchRate = calculateAngularRate(angles->pitch, prevAngles.pitch, dt);
    float yawRate = calculateAngularRate(angles->yaw, prevAngles.yaw, dt);

    // 更新上一次角度值
    prevAngles = *angles;

    // 外环：姿态控制
    // 计算角度期望值 (-1 到 1 映射到适当的角度范围，如 -30 到 30 度)
    float rollAngleSetpoint = rc->roll * 30.0;
    float pitchAngleSetpoint = rc->pitch * 30.0;
    float yawAngleSetpoint = last_yawSetPoint + (rc->yaw * 5.0 * dt); // 累积航向期望
    last_yawSetPoint = yawAngleSetpoint;

    // 规范化航向角到 -180 到 180 度
    if (yawAngleSetpoint > 180.0) yawAngleSetpoint -= 360.0;
    if (yawAngleSetpoint < -180.0) yawAngleSetpoint += 360.0;

    // 更新外环PID (角度控制)
//    printf("angles %f %f %f %f %f %f\r\n",
//           angles->roll, angles->pitch, angles->yaw,
//           rollAngleSetpoint, pitchAngleSetpoint, yawAngleSetpoint
//    );
    updatePIDController(&rollAnglePID, rollAngleSetpoint, angles->roll, dt);
    updatePIDController(&pitchAnglePID, pitchAngleSetpoint, angles->pitch, dt);
    updatePIDController(&yawAnglePID, yawAngleSetpoint, angles->yaw, dt);

    // 内环：角速度控制
    // 角速度期望值来自外环PID
    float rollRateSetpoint = rollAnglePID.output;
    float pitchRateSetpoint = pitchAnglePID.output;
    float yawRateSetpoint = yawAnglePID.output;

    // 更新内环PID (角速度控制)
    updatePIDController(&rollRatePID, rollRateSetpoint, rollRate, dt);
    updatePIDController(&pitchRatePID, pitchRateSetpoint, pitchRate, dt);
    updatePIDController(&yawRatePID, yawRateSetpoint, yawRate, dt);

    // 计算电机输出
    MotorOutputs outputs;
    calculateMotorOutputs(
            rc->throttle,
            rollRatePID.output,
            pitchRatePID.output,
            yawRatePID.output,
            &outputs
    );

    // 设置电机PWM
//    printf("motor %f %d %d %d %d %f %f %f\r\n",
//           dt ,outputs.motor1, outputs.motor2, outputs.motor3, outputs.motor4
//           , rollRatePID.output, pitchRatePID.output, yawRatePID.output);
    motor_ctrl(outputs.motor1, outputs.motor2, outputs.motor3, outputs.motor4);
}