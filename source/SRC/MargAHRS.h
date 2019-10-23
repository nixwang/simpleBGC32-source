//=====================================================================================================
// AHRS.h
// S.O.H. Madgwick
// 25th August 2010
//
// 1 June 2012 Modified by J. Ihlein
// 27 Aug  2012 Extensively modified to include G.K. Egan's accel confidence calculations and
//                                                          calculation efficiency updates
//=====================================================================================================
//
// See AHRS.c file for description.
//
//=====================================================================================================

#pragma once

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern float accConfidenceDecay[2];

extern float q0[2], q1[2], q2[2], q3[2];  // quaternion elements representing the estimated orientation

// auxiliary variables to reduce number of repeated operations
extern float q0q0[2], q0q1[2], q0q2[2], q0q3[2];
extern float q1q1[2], q1q2[2], q1q3[2];
extern float q2q2[2], q2q3[2];
extern float q3q3[2];

//---------------------------------------------------------------------------------------------------
// Function declaration
void calculateAccConfidence1(float accMag);
void calculateAccConfidence2(float accMag);

void MargAHRSinit1(float ax, float ay, float az, float mx, float my, float mz);
void MargAHRSinit2(float ax, float ay, float az, float mx, float my, float mz);

void MargAHRSupdate1(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    uint8_t magDataUpdate, float dt);
void MargAHRSupdate2(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    uint8_t magDataUpdate, float dt);

//=====================================================================================================
// End of file
//=====================================================================================================
