//
// Created by lsc on 25-3-17.
//
/* icm42688p.c */
#include "icm_42688p.h"
#include "stdio.h"

// 寄存器定义
#define REG_WHO_AM_I        0x75
#define REG_PWR_MGMT0       0x1F
#define REG_ACCEL_CONFIG0   0x3D
#define REG_GYRO_CONFIG0    0x3F
#define REG_TEMP_DATA1      0x09
#define REG_ACCEL_DATA_X1   0x1B
#define REG_GYRO_DATA_X1    0x25

// 软件I2C基本操作
#define SCL_HIGH()  HAL_GPIO_WritePin(hi2c->scl_port, hi2c->scl_pin, GPIO_PIN_SET)
#define SCL_LOW()   HAL_GPIO_WritePin(hi2c->scl_port, hi2c->scl_pin, GPIO_PIN_RESET)
#define SDA_HIGH()  HAL_GPIO_WritePin(hi2c->sda_port, hi2c->sda_pin, GPIO_PIN_SET)
#define SDA_LOW()   HAL_GPIO_WritePin(hi2c->sda_port, hi2c->sda_pin, GPIO_PIN_RESET)
#define SDA_READ()  HAL_GPIO_ReadPin(hi2c->sda_port, hi2c->sda_pin)

static SoftI2C_HandleTypeDef* hi2c;

static void I2C_Delay(void) {
    uint32_t delay = hi2c->delay_us;
    while(delay--) __NOP();
}

static void I2C_Start(void) {
    SDA_HIGH();
    SCL_HIGH();
    I2C_Delay();
    SDA_LOW();
    I2C_Delay();
    SCL_LOW();
}

static void I2C_Stop(void) {
    SDA_LOW();
    I2C_Delay();
    SCL_HIGH();
    I2C_Delay();
    SDA_HIGH();
    I2C_Delay();
}

static uint8_t I2C_WaitAck(void) {
    SDA_HIGH();
    I2C_Delay();
    SCL_HIGH();
    I2C_Delay();

    if(SDA_READ()) {
        SCL_LOW();
        return 1;
    }
    SCL_LOW();
    return 0;
}

static void I2C_SendByte(uint8_t byte) {
    for(uint8_t i = 0; i < 8; i++) {
        (byte & 0x80) ? SDA_HIGH() : SDA_LOW();
        I2C_Delay();
        SCL_HIGH();
        I2C_Delay();
        SCL_LOW();
        byte <<= 1;
    }
    I2C_WaitAck();
}

static uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t byte = 0;
    SDA_HIGH();

    for(uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        SCL_HIGH();
        I2C_Delay();
        if(SDA_READ()) byte |= 0x01;
        SCL_LOW();
        I2C_Delay();
    }

    // 发送ACK/NACK
    ack ? SDA_LOW() : SDA_HIGH();
    I2C_Delay();
    SCL_HIGH();
    I2C_Delay();
    SCL_LOW();
    SDA_HIGH();

    return byte;
}

static void I2C_WriteReg(uint8_t reg, uint8_t value) {
    I2C_Start();
    I2C_SendByte(ICM42688_I2C_ADDR & 0xFE);
    I2C_SendByte(reg);
    I2C_SendByte(value);
    I2C_Stop();
    HAL_Delay(1);
}

static uint8_t I2C_ReadReg(uint8_t reg) {
    uint8_t value;

    I2C_Start();
    I2C_SendByte(ICM42688_I2C_ADDR & 0xFE);
    I2C_SendByte(reg);
    I2C_Start();
    I2C_SendByte(ICM42688_I2C_ADDR | 0x01);
    value = I2C_ReadByte(0);
    I2C_Stop();

    return value;
}

static void I2C_ReadMultiReg(uint8_t reg, uint8_t* data, uint8_t len) {
    I2C_Start();
    I2C_SendByte(ICM42688_I2C_ADDR & 0xFE);
    I2C_SendByte(reg);
    I2C_Start();
    I2C_SendByte(ICM42688_I2C_ADDR | 0x01);

    while(len--) {
        *data++ = I2C_ReadByte(len ? 1 : 0);
    }

    I2C_Stop();
}

void ICM42688_Init(SoftI2C_HandleTypeDef* i2c_handle) {
    hi2c = i2c_handle;

    // 配置GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = hi2c->scl_pin;
    HAL_GPIO_Init(hi2c->scl_port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = hi2c->sda_pin;
    HAL_GPIO_Init(hi2c->sda_port, &GPIO_InitStruct);

    // 读取WHO_AM_I寄存器
    uint8_t who_am_i = I2C_ReadReg(REG_WHO_AM_I);
    printf("who am i = 0x%02X\r\n", who_am_i);

    // 传感器初始化
    HAL_Delay(100);
    I2C_WriteReg(REG_PWR_MGMT0, 0x0F);  // 启动所有传感器
    I2C_WriteReg(REG_ACCEL_CONFIG0, 0x05); // ±16g, 1kHz
    I2C_WriteReg(REG_GYRO_CONFIG0, 0x05);  // ±2000dps, 1kHz
}

uint8_t ICM42688_ReadData(SoftI2C_HandleTypeDef* hi2c, ICM42688_Data* data) {
    uint8_t buf[14];

    // 读取加速度计数据（6字节）
    I2C_ReadMultiReg(REG_ACCEL_DATA_X1, buf, 6);

    // 读取陀螺仪数据（6字节）
    I2C_ReadMultiReg(REG_GYRO_DATA_X1, &buf[6], 6);

    // 读取温度数据（2字节）
    I2C_ReadMultiReg(REG_TEMP_DATA1, &buf[12], 2);

    // 转换加速度计数据（16位补码）
    int16_t accel_x = (buf[0] << 8) | buf[1];
    int16_t accel_y = (buf[2] << 8) | buf[3];
    int16_t accel_z = (buf[4] << 8) | buf[5];

    // 转换陀螺仪数据（16位补码）
    int16_t gyro_x = (buf[6] << 8) | buf[7];
    int16_t gyro_y = (buf[8] << 8) | buf[9];
    int16_t gyro_z = (buf[10] << 8) | buf[11];

    // 转换温度数据
    int16_t temp = (buf[12] << 8) | buf[13];

    // 转换为物理量
    const float accel_scale = 16.0f / 32768.0f * 9.80665f; // m/s²
    const float gyro_scale = 2000.0f / 32768.0f * (3.1415926f / 180.0f); // rad/s

    data->accel_x = accel_x * accel_scale;
    data->accel_y = accel_y * accel_scale;
    data->accel_z = accel_z * accel_scale;

    data->gyro_x = gyro_x * gyro_scale;
    data->gyro_y = gyro_y * gyro_scale;
    data->gyro_z = gyro_z * gyro_scale;

    data->temp = (temp / 132.48f) + 25.0f; // 温度转换公式

    return 0;
}