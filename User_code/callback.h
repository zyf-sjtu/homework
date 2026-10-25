//
// Created by zyfsj on 2026/10/20.
//
#include "bmi088.h"
#include "imu.h"
#ifndef IMU_CALLBACK_H
#define IMU_CALLBACK_H

#ifdef __cplusplus
extern "C" {
#endif
    void userCallback();
    void userCalibrate();
#ifdef __cplusplus
}
#endif

#endif //IMU_CALLBACK_H
