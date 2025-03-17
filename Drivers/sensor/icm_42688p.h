//
// Created by lsc on 25-3-17.
//

#ifndef MFLY_ICM_42688P_H
#define MFLY_ICM_42688P_H

#include "stm32g4xx_hal.h"

#define ICM42688_I2C_ADDR 0x68

typedef struct {
    float accel_x;  // m/s²
    float accel_y;
    float accel_z;
    float gyro_x;   // rad/s
    float gyro_y;
    float gyro_z;
    float temp;     // °C
} ICM42688_Data;

typedef struct {
    GPIO_TypeDef* scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef* sda_port;
    uint16_t sda_pin;
    uint32_t delay_us;
} SoftI2C_HandleTypeDef;

// 初始化函数
void ICM42688_Init(SoftI2C_HandleTypeDef* hi2c);
uint8_t ICM42688_ReadData(SoftI2C_HandleTypeDef* hi2c, ICM42688_Data* data);

#endif //MFLY_ICM_42688P_H
