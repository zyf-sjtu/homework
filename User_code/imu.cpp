//
// Created by zyfsj on 2026/10/20.
//

#include "imu.h"
#include <string.h>
#include "bmi088.h"
#include <cmath>
#include <math.h>

#define MY_PI 3.14159265358979323846f

IMU::IMU(float alpha):alpha_(alpha){;}
void IMU::reset()
{
    memset(acc_data_,0,6);
    memset(gyro_data_,0,6);
    ACC_Z_MSB_=0;
    ACC_Y_MSB_=0;
    ACC_X_MSB_=0;
    ACC_Y_LSB_=0;
    ACC_X_LSB_=0;
    ACC_Z_LSB_=0;
    RATE_Z_MSB_=0;
    RATE_Y_MSB_=0;
    RATE_X_MSB_=0;
    RATE_Y_LSB_=0;
    RATE_X_LSB_=0;
    RATE_Z_LSB_=0;
}
void IMU::handleSensor()
{
    decode();
    calc();
    sensorDataTransform();
    updateQuaternion();
    getGyroAngles();
    getAccAngles();
    complementaryFilter();
}
bool IMU::hasfeedback()
{
    return received_;
}
void IMU::decode()
{
    received_=true;
    bmi088_accel_read_reg(0x12,acc_data_,6);
    bmi088_gyro_read_reg(0x02,gyro_data_,6);
    ACC_X_LSB_=acc_data_[0];
    ACC_X_MSB_=acc_data_[1];
    ACC_Y_LSB_=acc_data_[2];
    ACC_Y_MSB_=acc_data_[3];
    ACC_Z_LSB_=acc_data_[4];
    ACC_Z_MSB_=acc_data_[5];
    RATE_X_LSB_=gyro_data_[0];
    RATE_X_MSB_=gyro_data_[1];
    RATE_Y_LSB_=gyro_data_[2];
    RATE_Y_MSB_=gyro_data_[3];
    RATE_Z_LSB_=gyro_data_[4];
    RATE_Z_MSB_=gyro_data_[5];
}
void IMU::calc()
{
    acc_x_int_=(int16_t)(ACC_X_LSB_|((uint16_t)ACC_X_MSB_<<8));
    acc_y_int_=(int16_t)(ACC_Y_LSB_|((uint16_t)ACC_Y_MSB_<<8));
    acc_z_int_=(int16_t)(ACC_Z_LSB_|((uint16_t)ACC_Z_MSB_<<8));
    gyro_x_int_=(int16_t)(RATE_X_LSB_|((uint16_t)RATE_X_MSB_<<8));
    gyro_y_int_=(int16_t)(RATE_Y_LSB_|((uint16_t)RATE_Y_MSB_<<8));
    gyro_z_int_=(int16_t)(RATE_Z_LSB_|((uint16_t)RATE_Z_MSB_<<8));
    acc_[0]=(float)acc_x_int_/32768.0f*6.0f*9.8f;
    acc_[1]=(float)acc_y_int_/32768.0f*6.0f*9.8f;
    acc_[2]=(float)acc_z_int_/32768.0f*6.0f*9.8f;
    gyro_[0]=(float)gyro_x_int_/32768.0f*2000.0f/180*MY_PI;
    gyro_[1]=(float)gyro_y_int_/32768.0f*2000.0f/180*MY_PI;
    gyro_[2]=(float)gyro_z_int_/32768.0f*2000.0f/180*MY_PI;
}

void IMU::getGyroAngles()
{
    float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;

    float R[3][3];
    R[0][0] = 1.0f - 2.0f * (yy + zz);
    R[0][1] = 2.0f * (xy - wz);
    R[0][2] = 2.0f * (xz + wy);

    R[1][0] = 2.0f * (xy + wz);
    R[1][1] = 1.0f - 2.0f * (xx + zz);
    R[1][2] = 2.0f * (yz - wx);

    R[2][0] = 2.0f * (xz - wy);
    R[2][1] = 2.0f * (yz + wx);
    R[2][2] = 1.0f - 2.0f * (xx + yy);
    float sinp = -R[2][0];
    if (fabsf(sinp) >= 1.0f) {
        pitch_gyro_ = copysignf(3.14159265f / 2.0f, sinp); // 限制在 ±90度
    } else {
        pitch_gyro_ = asinf(sinp);
    }

    // 2. Roll (X轴) - 使用 atan2，能自动处理四个象限
    roll_gyro_ = atan2f(R[2][1], R[2][2]);

    // 3. Yaw (Z轴) - 同样使用 atan2
    yaw_gyro_ = atan2f(R[1][0], R[0][0]);
}

void IMU::getAccAngles()
{
    float ax = acc_sensor_[0];
    float ay = acc_sensor_[1];
    float az = acc_sensor_[2];

    // 保护机制：防止数据全为0或者开出负数
    float norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;

    // 1. 计算 Roll (横滚角)
    // 使用 atan2(y, x) 形式，直接得出弧度值
    roll_accel_ = atan2f(ay, az);

    // 2. 计算 Pitch (俯仰角)
    // 先算出 YZ 平面上的投影长度
    float yz_norm = sqrtf(ay * ay + az * az);
    pitch_accel_ = atan2f(-ax, yz_norm);
}

void IMU::sensorDataTransform(){
    for (uint8_t i = 0; i < 3; i++) {
        gyro_sensor_[i] = gyro_[0] * R_imu_[i][0] +
                          gyro_[1] * R_imu_[i][1] +
                          gyro_[2] * R_imu_[i][2] - gyro_bias_[i];
        acc_sensor_[i] = acc_sen_ * (acc_[0] * R_imu_[i][0] +
                                         acc_[1] * R_imu_[i][1] +
                                         acc_[2] * R_imu_[i][2]) -acc_bias_[i];
    }
}

void IMU::updateQuaternion()
{
    float half_dt=0.5f*dt_;
    // dq = 0.5 * q ⊗ (0, w)
    dq_[0] = (-q_[1] * gyro_sensor_[0] - q_[2] * gyro_sensor_[1] - q_[3] * gyro_sensor_[2]) * half_dt;
    dq_[1] = ( q_[0] * gyro_sensor_[0] + q_[2] * gyro_sensor_[2] - q_[3] * gyro_sensor_[1]) * half_dt;
    dq_[2] = ( q_[0] * gyro_sensor_[1] - q_[1] * gyro_sensor_[2] + q_[3] * gyro_sensor_[0]) * half_dt;
    dq_[3] = ( q_[0] * gyro_sensor_[2] + q_[1] * gyro_sensor_[1] - q_[2] * gyro_sensor_[0]) * half_dt;

    for (uint8_t i = 0; i < 4; i++)
    {
        q_[i] += dq_[i];
    }

    float mod=sqrt(q_[0]*q_[0]+q_[1]*q_[1]+q_[2]*q_[2]+q_[3]*q_[3]);
    for (uint8_t i = 0; i < 4; i++)
    {
        q_[i] /= mod;
    }
}

void IMU::complementaryFilter()
{
    yaw_+=gyro_sensor_[2]*dt_;
    pitch_=alpha_*(pitch_+dt_*gyro_sensor_[1])+(1-alpha_)*pitch_accel_;
    roll_=alpha_*(roll_+dt_*gyro_sensor_[0])+(1-alpha_)*roll_accel_;
}

void IMU::calibrate()
{
    float sum_x=0.0, sum_y=0.0 ,sum_z=0.0;

    for (uint16_t i = 0; i < 2000; i++)
    {
        decode();
        calc();
        sensorDataTransform();
        sum_x += gyro_sensor_[0];
        sum_y += gyro_sensor_[1];
        sum_z += gyro_sensor_[2];
        HAL_Delay(1);
    }
    gyro_bias_[0]=sum_x/2000.0f;
    gyro_bias_[1]=sum_y/2000.0f;
    gyro_bias_[2]=sum_z/2000.0f;
}