//
// Created by zyfsj on 2026/10/20.
//

#include "callback.h"
#include "IMU.h"
#include "bmi088.h"

IMU imu(0.8);
extern "C" void userCallback()
{
    imu.handleSensor();
    imu.updateQuaternion();
}

extern "C" void userCalibrate()
{
    imu.calibrate();
}