//
// Created by zyfsj on 2026/10/20.
//
#include "main.h"
#ifndef IMU_IMU_H
#define IMU_IMU_H

class IMU
{
    public:
    explicit IMU(float alpha);
    void reset(void);
    void handleSensor(void);
    void updateQuaternion(void);
    bool hasfeedback(void);
    void calibrate(void);
    private:
    const float R_imu_[3][3] = {{-1.0f, 0.0f, 0.0f},
                         {0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, -1.0f}};
    bool received_=false;
    uint8_t acc_data_[6];
    uint8_t gyro_data_[6];
    volatile uint8_t  ACC_Z_MSB_=0;
    volatile uint8_t  ACC_Y_MSB_=0;
    volatile uint8_t  ACC_X_MSB_=0;
    volatile uint8_t  ACC_Y_LSB_=0;
    volatile uint8_t  ACC_X_LSB_=0;
    volatile uint8_t  ACC_Z_LSB_=0;
    volatile uint8_t  RATE_Z_MSB_=0;
    volatile uint8_t  RATE_Y_MSB_=0;
    volatile uint8_t  RATE_X_MSB_=0;
    volatile uint8_t  RATE_Y_LSB_=0;
    volatile uint8_t  RATE_X_LSB_=0;
    volatile uint8_t  RATE_Z_LSB_=0;
    volatile int16_t acc_x_int_=0, acc_y_int_=0, acc_z_int_=0;
    volatile int16_t gyro_x_int_=0, gyro_y_int_=0, gyro_z_int_=0;
    volatile float acc_[3];
    volatile float gyro_[3];
    volatile float acc_sensor_[3];
    volatile float gyro_sensor_[3];
    float acc_sen_=1.0f;
    float gyro_bias_[3]={0.0f, 0.0f, 0.0f};
    float acc_bias_[3]={0.0f, 0.0f, 0.0f};
    float roll_gyro_=0.0f;
    float pitch_gyro_=0.0f;
    float yaw_gyro_=0.0f;
    float roll_accel_=0.0f;
    float pitch_accel_=0.0f;
    float yaw_=0.0f;
    float pitch_=0.0f;
    float roll_=0.0f;
    volatile float q_[4]={1.0f, 0.0f, 0.0f, 0.0f};
    volatile float dq_[4]={0.0f, 0.0f, 0.0f, 0.0f};
    float dt_=0.001;
    float alpha_=0.0f;
    void decode(void);
    void calc(void);
    void sensorDataTransform(void);
    void getGyroAngles(void);
    void getAccAngles(void);
    void complementaryFilter(void);
};

#endif //IMU_IMU_H
