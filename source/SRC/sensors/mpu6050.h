/*
  Sept 2013

  bgc32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Brushless Gimbal Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the EvvGC Brushless Gimbal Controller Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

//#pragma once
#include "stm32f10x.h"
///////////////////////////////////////////////////////////////////////////////

#define MPU6050_ADDRESS2             0x68 // camera IMU
#define MPU6050_ADDRESS1             0x69 // Main Board IMU

#define MPU6050_CONFIG              0x1A

#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03

#define ACCEL_SCALE_FACTOR 0.00119708f  // (1/8192) * 9.8065  (8192 LSB = 1 G)
#define GYRO_SCALE_FACTOR  0.00026646f  // (1/65.5) * pi/180   (65.5 LSB = 1 DPS)

///////////////////////////////////////////////////////////////////////////////
// MPU6050 Variables
///////////////////////////////////////////////////////////////////////////////

extern float   accelOneG;

extern float   accelTCBias[3];

extern int16_t accelData500Hz[2][3];

extern int16andUint8_t rawAccel[2][3];

///////////////////////////////////////

extern float gyroRTBias[2][3];

extern float gyroTCBias[2][3];

extern int16_t gyroData500Hz[2][3];

extern int16andUint8_t rawGyro[2][3];

///////////////////////////////////////

extern uint8_t mpu6050Calibrating;

extern float   mpu6050Temperature;

extern int16andUint8_t rawMPU6050Temperature[2];

///////////////////////////////////////////////////////////////////////////////
// MPU6050 Initialization
///////////////////////////////////////////////////////////////////////////////

void initMPU6050_i2c2(void);
void initMPU6050_i2c1(void);

///////////////////////////////////////////////////////////////////////////////
// Read MPU6050
///////////////////////////////////////////////////////////////////////////////

void readMPU60501(void);

void readMPU60502(void);

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6050 Runtime Data
///////////////////////////////////////////////////////////////////////////////

void computeMPU6050RTData1(void);

void computeMPU6050RTData2(void);

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6050 Temperature Compensation Bias
///////////////////////////////////////////////////////////////////////////////

void computeMPU6050TCBias1(void);

void computeMPU6050TCBias2(void);

///////////////////////////////////////////////////////////////////////////////
// Orient IMU
///////////////////////////////////////////////////////////////////////////////

void orientIMU(void);

///////////////////////////////////////////////////////////////////////////////
