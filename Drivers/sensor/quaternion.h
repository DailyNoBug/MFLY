//
// Created by lsc on 25-3-26.
//

#ifndef MFLY_QUATERNION_H
#define MFLY_QUATERNION_H
#include <math.h>
#include <stdint.h>
#include <i2c.h>
#include <stdio.h>

#define GY95_T_ADDR 0x52
#define START_ADDR  0x08
#define ADDR_SIZE   35

// 定义四元数结构体
typedef struct {
    float w, x, y, z;
} Quaternion;

// IMU数据结构体
typedef struct {
    int16_t acc_x, acc_y, acc_z;       // 欧拉角（度）
    int16_t gyro_x, gyro_y, gyro_z;    // 角速度（度/秒）
    int16_t mag_x, mag_y, mag_z;       // 磁场强度（高斯）
    uint8_t level;                     // 磁场校准精度 100 best
    float temperature;              // 温度（摄氏度）
    Quaternion Q;                      // 姿态四元数
} IMU_Data;



// 定义欧拉角结构体
typedef struct {
    float roll;   // 横滚（绕X轴，单位：度）
    float pitch;  // 俯仰（绕Y轴，单位：度）
    float yaw;    // 偏航（绕Z轴，单位：度）
} EulerAngles;



EulerAngles QuaternionToEuler(const Quaternion *q);
HAL_StatusTypeDef ReadRegister(uint8_t devAddr, uint8_t regAddr, uint8_t *pData);
HAL_StatusTypeDef WriteRegister(uint8_t devAddr, uint8_t regAddr, uint8_t data);
HAL_StatusTypeDef ReadMultiRegister(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint16_t size);
void ReadIMUData(IMU_Data *imu);
void caculate_angle(void);
#endif //MFLY_QUATERNION_H
