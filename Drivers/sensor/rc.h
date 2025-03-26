//
// Created by lsc on 25-3-26.
//

#ifndef MFLY_RC_H
#define MFLY_RC_H
#include "stdint.h"
// 遥控器输入结构体
typedef struct {
    float throttle;  // 油门（0-100%）
    float roll;      // 横滚（-100%~+100%）
    float pitch;     // 俯仰（-100%~+100%）
    float yaw;       // 偏航（-100%~+100%）
    uint8_t lock;    // 上锁
    uint8_t mode;    // 模式
} RC_Input;

#endif //MFLY_RC_H
