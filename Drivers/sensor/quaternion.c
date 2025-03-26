//
// Created by lsc on 25-3-26.
//

#include "quaternion.h"

IMU_Data imu;
EulerAngles eulera;
// 四元数转欧拉角（ZYX顺序）
EulerAngles QuaternionToEuler(const Quaternion *q) {
    EulerAngles euler;

    // 计算俯仰角（Pitch）
    float sin_p = 2.0f * (q->w * q->y - q->z * q->x);
    sin_p = (sin_p > 1.0f) ? 1.0f : (sin_p < -1.0f) ? -1.0f : sin_p; // 限制范围[-1, 1]
    euler.pitch = asinf(sin_p) * 180.0f / M_PI;  // 转为角度

    // 计算横滚角（Roll）和偏航角（Yaw）
    float roll_numerator = 2.0f * (q->w * q->x + q->y * q->z);
    float roll_denominator = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    euler.roll = atan2f(roll_numerator, roll_denominator) * 180.0f / M_PI;

    float yaw_numerator = 2.0f * (q->w * q->z + q->x * q->y);
    float yaw_denominator = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    euler.yaw = atan2f(yaw_numerator, yaw_denominator) * 180.0f / M_PI;

    return euler;
}

HAL_StatusTypeDef ReadRegister(uint8_t devAddr, uint8_t regAddr, uint8_t *pData)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            devAddr << 1,    // 7位地址左移1位
                            regAddr,
                            I2C_MEMADD_SIZE_8BIT,
                            pData,
                            1,
                            100);
}

HAL_StatusTypeDef WriteRegister(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             devAddr << 1,   // 7位地址左移1位
                             regAddr,
                             I2C_MEMADD_SIZE_8BIT,
                             &data,
                             1,
                             100);
}

HAL_StatusTypeDef ReadMultiRegister(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint16_t size)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            devAddr << 1,    // 7位地址左移1位
                            regAddr,
                            I2C_MEMADD_SIZE_8BIT,
                            pData,
                            size,
                            100);
}

void ReadIMUData(IMU_Data *imu) {
    static uint8_t data[35];
    ReadMultiRegister(GY95_T_ADDR, START_ADDR, data, ADDR_SIZE);

    // 加速度计数据
    imu->acc_x = (int16_t)(data[1] << 8 | data[0]);
    imu->acc_y = (int16_t)(data[3] << 8 | data[2]);
    imu->acc_z = (int16_t)(data[5] << 8 | data[4]);

    // 陀螺仪数据
    imu->gyro_x = (int16_t)(data[7] << 8 | data[6]);
    imu->gyro_y = (int16_t)(data[9] << 8 | data[8]);
    imu->gyro_z = (int16_t)(data[11] << 8 | data[10]);

    imu->level = (uint8_t)data[18];

    imu->temperature = (float )(data[20] << 8 | data[19]) / 100.0f;

    imu->mag_x = (int16_t)(data[22] << 8 | data[21]);
    imu->mag_y = (int16_t)(data[24] << 8 | data[23]);
    imu->mag_z = (int16_t)(data[26] << 8 | data[25]);

    // 四元数数据
    imu->Q.w = (int16_t)(data[28] << 8 | data[27]) / 10000.0f;
    imu->Q.x = (int16_t)(data[30] << 8 | data[29]) / 10000.0f;
    imu->Q.y = (int16_t)(data[32] << 8 | data[31]) / 10000.0f;
    imu->Q.z = (int16_t)(data[34] << 8 | data[33]) / 10000.0f;
}
void caculate_angle(void)
{
    ReadIMUData(&imu);
    EulerAngles euler = QuaternionToEuler(&imu.Q);
    eulera.roll = euler.roll;
    eulera.pitch = euler.pitch;
    eulera.yaw = euler.yaw;
//    printf("roll = %f, pitch = %f, yaw = %f\n", eulera.roll, eulera.pitch, eulera.yaw);
    return ;
}