///=====================================================================================================
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//
//  1 June 2012 Modified by J. Ihlein
// 27 Aug  2012 Extensively modified to include G.K. Egan's accel confidence calculations and
//                                                          calculation efficiency updates
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Header files

#include "board.h"

//----------------------------------------------------------------------------------------------------
// Variable definitions

float exAcc[2]    = {0.0f,0.0f},    eyAcc[2] = {0.0f,0.0f},    ezAcc[2] = {0.0f,0.0f}; // accel error
float exAccInt[2] = {0.0f,0.0f}, eyAccInt[2] = {0.0f,0.0f}, ezAccInt[2] = {0.0f,0.0f}; // accel integral error

float exMag[2]    = {0.0f,0.0f}, eyMag[2]    = {0.0f,0.0f}, ezMag[2]    = {0.0f,0.0f}; // mag error
float exMagInt[2] = {0.0f,0.0f}, eyMagInt[2] = {0.0f,0.0f}, ezMagInt[2] = {0.0f,0.0f}; // mag integral error

float kpAcc[2], kiAcc[2];

float q0[2] = {0.0f,0.0f}, q1[2] = {0.0f,0.0f}, q2[2] = {0.0f,0.0f}, q3[2] = {0.0f,0.0f};

// auxiliary variables to reduce number of repeated operations
float q0q0[2], q0q1[2], q0q2[2], q0q3[2];
float q1q1[2], q1q2[2], q1q3[2];
float q2q2[2], q2q3[2];
float q3q3[2];

float halfT[2];

uint8_t MargAHRSinitialized1 = false;
uint8_t MargAHRSinitialized2 = false;

//----------------------------------------------------------------------------------------------------

float accConfidenceDecay[2] = {0.0f, 0.0f};
float accConfidence[2]      = {1.0f, 1.0f};

#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)

void calculateAccConfidence1(float accMag)
{
    // G.K. Egan (C) computes confidence in accelerometers when
    // aircraft is being accelerated over and above that due to gravity

    static float accMagP = 1.0f;

    accMag /= accelOneG;  // HJI Added to convert MPS^2 to G's

    accMag  = HardFilter(accMagP, accMag);
    accMagP = accMag;

    accConfidence[0] = constrain(1.0f - (accConfidenceDecay[0] * sqrt(fabs(accMag - 1.0f))), 0.0f, 1.0f);
}

void calculateAccConfidence2(float accMag)
{
    // G.K. Egan (C) computes confidence in accelerometers when
    // aircraft is being accelerated over and above that due to gravity

    static float accMagP = 1.0f;

    accMag /= accelOneG;  // HJI Added to convert MPS^2 to G's

    accMag  = HardFilter(accMagP, accMag);
    accMagP = accMag;

    accConfidence[1] = constrain(1.0f - (accConfidenceDecay[1] * sqrt(fabs(accMag - 1.0f))), 0.0f, 1.0f);
}
//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Initialization
//====================================================================================================
//航姿参考系统初始化
void MargAHRSinit1(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

	//使用加速度数据计算欧拉角 ，滚转角和俯仰角
    initialRoll  = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

	//对欧拉角进行余弦和正弦计算，分别把计算结果保存下来
    cosRoll  = cosf(initialRoll);
    sinRoll  = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = 1.0f;  // HJI mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = 0.0f;  // HJI my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);//解算航向角

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

		//得到四元数
    q0[0] = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1[0] = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2[0] = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3[0] = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

		//把计算参考方向用到的值先都计算好,减少重复计算,因为MargAHRSupdate函数里面要用到。
    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0[0] = q0[0] * q0[0];
    q0q1[0] = q0[0] * q1[0];
    q0q2[0] = q0[0] * q2[0];
    q0q3[0] = q0[0] * q3[0];
    q1q1[0] = q1[0] * q1[0];
    q1q2[0] = q1[0] * q2[0];
    q1q3[0] = q1[0] * q3[0];
    q2q2[0] = q2[0] * q2[0];
    q2q3[0] = q2[0] * q3[0];
    q3q3[0] = q3[0] * q3[0];
}

void MargAHRSinit2(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

	//使用加速度数据计算欧拉角 ，滚转角和俯仰角
    initialRoll  = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

	//对欧拉角进行余弦和正弦计算，分别把计算结果保存下来
    cosRoll  = cosf(initialRoll);
    sinRoll  = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = 1.0f;  // HJI mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = 0.0f;  // HJI my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);//解算航向角

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

		//得到四元数
    q0[1] = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1[1] = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2[1] = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3[1] = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

		//把计算参考方向用到的值先都计算好,减少重复计算,因为MargAHRSupdate函数里面要用到。
    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0[1] = q0[1] * q0[1];
    q0q1[1] = q0[1] * q1[1];
    q0q2[1] = q0[1] * q2[1];
    q0q3[1] = q0[1] * q3[1];
    q1q1[1] = q1[1] * q1[1];
    q1q2[1] = q1[1] * q2[1];
    q1q3[1] = q1[1] * q3[1];
    q2q2[1] = q2[1] * q2[1];
    q2q3[1] = q2[1] * q3[1];
    q3q3[1] = q3[1] * q3[1];
}
//====================================================================================================
// Function
//====================================================================================================
//航姿参考系统更新
void MargAHRSupdate1(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    uint8_t magDataUpdate, float dt)
{
  
    float norm, normR;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float q0i, q1i, q2i, q3i;

    //-------------------------------------------
    
    // HJI&& (magDataUpdate == true)
    if ((MargAHRSinitialized1 == false) )
    {
			//如果航姿参考系统参数还没有初始化过，那么执行AHRS初始化
        MargAHRSinit1(ax, ay, az, mx, my, mz);

        // cliPrintF(" mag10HzPitch %d ", (int)(sensors.mag10Hz[0]*100));
        // cliPrintF(" mag10HzROLL %d ", (int)(sensors.mag10Hz[1]*100));
        // cliPrintF(" mag10HzYAW %d \n", (int)(sensors.mag10Hz[2]*100));

        MargAHRSinitialized1 = true;//标记航姿参考系统参数已经初始化过
    }

    //-------------------------------------------

    if (MargAHRSinitialized1 == true)//如果航姿参考系统参数已经初始化过
    {
        halfT[0] = dt * 0.5f;//半周期，求解四元数微分方程时用得到。

        norm = sqrt(SQR(ax) + SQR(ay) + SQR(az));//加速度归一化

        if (norm != 0.0f)//如果归一化后的模等于0 ，那么说明加速度数据或者传感器不正常，正常情况下 归一化后的结果恒等于 1.0 ，这是重点。
        {
            calculateAccConfidence1(norm);//由于处于运动状态，所有要计算加速度数据归一化后的可信度
            kpAcc[0] = eepromConfig.KpAcc * accConfidence[0]; //加速度比例系数 * 可信度
            kiAcc[0] = eepromConfig.KiAcc * accConfidence[0];//加速度积分系数 * 可信度

            normR = 1.0f / norm; //加速度归一化
            ax *= normR;
            ay *= normR;
            az *= normR;

            // estimated direction of gravity (v)
            vx = 2.0f * (q1q3[0] - q0q2[0]);//计算方向余弦矩阵
            vy = 2.0f * (q0q1[0] + q2q3[0]);
            vz = q0q0[0] - q1q1[0] - q2q2[0] + q3q3[0];

            // error is sum of cross product between reference direction
            // of fields and direction measured by sensors
					//误差是由传感器测量的参考方向与方向之间的叉积,由此
					//得到一个误差向量，通过这个误差向量来修正陀螺仪数据。
            exAcc[0] = vy * az - vz * ay; 
            eyAcc[0] = vz * ax - vx * az;
            ezAcc[0] = vx * ay - vy * ax;


            gx += exAcc[0] * kpAcc[0];//比例增益控制加速度计的收敛速度
            gy += eyAcc[0] * kpAcc[0];
            gz += ezAcc[0] * kpAcc[0];

            if (kiAcc[0] > 0.0f)//用积分增益控制陀螺仪的偏差收敛速率
            {
                exAccInt[0] += exAcc[0] * kiAcc[0];
                eyAccInt[0] += eyAcc[0] * kiAcc[0];
                ezAccInt[0] += ezAcc[0] * kiAcc[0];

                gx += exAccInt[0];
                gy += eyAccInt[0];
                gz += ezAccInt[0];
            }
        }

        //-------------------------------------------

        norm = sqrt(SQR(mx) + SQR(my) + SQR(mz));//三轴磁力计归一化

        if ((magDataUpdate == true) && (norm != 0.0f))//如果入口参数magDataUpdate == true并且归一化的结果norm不是0，才对磁力计数据进行更新计算
        {
            normR = 1.0f / norm;//三轴磁场归一化
            mx *= normR;
            my *= normR;
            mz *= normR;

            // compute reference direction of flux
					//计算参考方向
            hx = 2.0f * (mx * (0.5f - q2q2[0] - q3q3[0]) + my * (q1q2[0] - q0q3[0]) + mz * (q1q3[0] + q0q2[0]));

            hy = 2.0f * (mx * (q1q2[0] + q0q3[0]) + my * (0.5f - q1q1[0] - q3q3[0]) + mz * (q2q3[0] - q0q1[0]));

            hz = 2.0f * (mx * (q1q3[0] - q0q2[0]) + my * (q2q3[0] + q0q1[0]) + mz * (0.5f - q1q1[0] - q2q2[0]));

            bx = sqrt((hx * hx) + (hy * hy));

            bz = hz;

            // estimated direction of flux (w)
					//根据参考方向估计云台机体方向
            wx = 2.0f * (bx * (0.5f - q2q2[0] - q3q3[0]) + bz * (q1q3[0] - q0q2[0]));

            wy = 2.0f * (bx * (q1q2[0] - q0q3[0]) + bz * (q0q1[0] + q2q3[0]));

            wz = 2.0f * (bx * (q0q2[0] + q1q3[0]) + bz * (0.5f - q1q1[0] - q2q2[0]));

            exMag[0] = my * wz - mz * wy;//三轴磁场和估计方向进行叉积运算,计算估计方向与三轴磁场的偏差
            eyMag[0] = mz * wx - mx * wz;
            ezMag[0] = mx * wy - my * wx;

            // use un-extrapolated old values between magnetometer updates
            // dubious as dT does not apply to the magnetometer calculation so
            // time scaling is embedded in KpMag and KiMag
						//使用估计的旧值与磁力计值进行更新，dT不能应用在磁力计计算中，因此时间被嵌入在KpMag 和 KiMag里面
            gx += exMag[0] * eepromConfig.KpMag;//比例增益控制磁强计收敛速度
            gy += eyMag[0] * eepromConfig.KpMag;
            gz += ezMag[0] * eepromConfig.KpMag;

			
            if (eepromConfig.KiMag > 0.0f)//用积分增益控制陀螺仪的偏差收敛速率
            {
                exMagInt[0] += exMag[0] * eepromConfig.KiMag;
                eyMagInt[0] += eyMag[0] * eepromConfig.KiMag;
                ezMagInt[0] += ezMag[0] * eepromConfig.KiMag;

                gx += exMagInt[0];
                gy += eyMagInt[0];
                gz += ezMagInt[0];
            }
        }

        //-------------------------------------------

        // integrate quaternion rate
			 //四元数微分方程，其中halfT为测量周期，g为陀螺仪角速度，其余都是已知量，这里使用了一阶龙格库塔法求解四元数微分方程。
        q0i = (-q1[0] * gx - q2[0] * gy - q3[0] * gz) * halfT[0];
        q1i = (q0[0] * gx + q2[0] * gz - q3[0] * gy) * halfT[0];
        q2i = (q0[0] * gy - q1[0] * gz + q3[0] * gx) * halfT[0];
        q3i = (q0[0] * gz + q1[0] * gy - q2[0] * gx) * halfT[0];
        q0[0] += q0i;
        q1[0] += q1i;
        q2[0] += q2i;
        q3[0] += q3i;

        // normalise quaternion
				//四元数归一化，为什么又要归一化呢？这是因为引入了误差向量后四元数失去了规范性了(模不等于1了),所以要重新归一化
        normR = 1.0f / sqrt(q0[0] * q0[0] + q1[0] * q1[0] + q2[0] * q2[0] + q3[0] * q3[0]);
        q0[0] *= normR;
        q1[0] *= normR;
        q2[0] *= normR;
        q3[0] *= normR;

        // auxiliary variables to reduce number of repeated operations
				//把计算参考方向用到的值先都计算好,减少下面计算欧拉角时候的重复计算。
        q0q0[0] = q0[0] * q0[0];
        q0q1[0] = q0[0] * q1[0];
        q0q2[0] = q0[0] * q2[0];
        q0q3[0] = q0[0] * q3[0];
        q1q1[0] = q1[0] * q1[0];
        q1q2[0] = q1[0] * q2[0];
        q1q3[0] = q1[0] * q3[0];
        q2q2[0] = q2[0] * q2[0];
        q2q3[0] = q2[0] * q3[0];
        q3q3[0] = q3[0] * q3[0];

				//最后根据四元数方向余弦阵和欧拉角的转换关系，把四元数转换成欧拉角
        sensors[0].margAttitude500Hz[ROLL ] = atan2f(2.0f * (q0q1[0] + q2q3[0]), q0q0[0] - q1q1[0] - q2q2[0] + q3q3[0]);
        sensors[0].margAttitude500Hz[PITCH] = -asinf(2.0f * (q1q3[0] - q0q2[0]));
        sensors[0].margAttitude500Hz[YAW  ] = atan2f(2.0f * (q1q2[0] + q0q3[0]), q0q0[0] + q1q1[0] - q2q2[0] - q3q3[0]);

       cliPrintF("\r\n ROLL[0] %d ", (int)(sensors[0].margAttitude500Hz[ROLL]*100));
       cliPrintF(" PITCH[0] %d ", (int)(sensors[0].margAttitude500Hz[PITCH]*100));
       cliPrintF(" YAW[0] %d ", (int)(sensors[0].margAttitude500Hz[YAW]*100));

    }
}

void MargAHRSupdate2(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    uint8_t magDataUpdate, float dt)
{
  
    float norm, normR;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float q0i, q1i, q2i, q3i;

    //-------------------------------------------
    
    // HJI&& (magDataUpdate == true)
    if ((MargAHRSinitialized2 == false) )
    {
			//如果航姿参考系统参数还没有初始化过，那么执行AHRS初始化
        MargAHRSinit2(ax, ay, az, mx, my, mz);

        // cliPrintF(" mag10HzPitch %d ", (int)(sensors.mag10Hz[0]*100));
        // cliPrintF(" mag10HzROLL %d ", (int)(sensors.mag10Hz[1]*100));
        // cliPrintF(" mag10HzYAW %d \n", (int)(sensors.mag10Hz[2]*100));

        MargAHRSinitialized2 = true;//标记航姿参考系统参数已经初始化过
    }

    //-------------------------------------------

    if (MargAHRSinitialized2 == true)//如果航姿参考系统参数已经初始化过
    {
        halfT[1] = dt * 0.5f;//半周期，求解四元数微分方程时用得到。

        norm = sqrt(SQR(ax) + SQR(ay) + SQR(az));//加速度归一化

        if (norm != 0.0f)//如果归一化后的模等于0 ，那么说明加速度数据或者传感器不正常，正常情况下 归一化后的结果恒等于 1.0 ，这是重点。
        {
            calculateAccConfidence2(norm);//由于处于运动状态，所有要计算加速度数据归一化后的可信度
            kpAcc[1] = eepromConfig.KpAcc * accConfidence[1]; //加速度比例系数 * 可信度
            kiAcc[1] = eepromConfig.KiAcc * accConfidence[1];//加速度积分系数 * 可信度

            normR = 1.0f / norm; //加速度归一化
            ax *= normR;
            ay *= normR;
            az *= normR;

            // estimated direction of gravity (v)
            vx = 2.0f * (q1q3[1] - q0q2[1]);//计算方向余弦矩阵
            vy = 2.0f * (q0q1[1] + q2q3[1]);
            vz = q0q0[1] - q1q1[1] - q2q2[1] + q3q3[1];

            // error is sum of cross product between reference direction
            // of fields and direction measured by sensors
					//误差是由传感器测量的参考方向与方向之间的叉积,由此
					//得到一个误差向量，通过这个误差向量来修正陀螺仪数据。
            exAcc[1] = vy * az - vz * ay; 
            eyAcc[1] = vz * ax - vx * az;
            ezAcc[1] = vx * ay - vy * ax;


            gx += exAcc[1] * kpAcc[1];//比例增益控制加速度计的收敛速度
            gy += eyAcc[1] * kpAcc[1];
            gz += ezAcc[1] * kpAcc[1];

            if (kiAcc[1] > 0.0f)//用积分增益控制陀螺仪的偏差收敛速率
            {
                exAccInt[1] += exAcc[1] * kiAcc[1];
                eyAccInt[1] += eyAcc[1] * kiAcc[1];
                ezAccInt[1] += ezAcc[1] * kiAcc[1];

                gx += exAccInt[1];
                gy += eyAccInt[1];
                gz += ezAccInt[1];
            }
        }

        //-------------------------------------------

        norm = sqrt(SQR(mx) + SQR(my) + SQR(mz));//三轴磁力计归一化

        if ((magDataUpdate == true) && (norm != 0.0f))//如果入口参数magDataUpdate == true并且归一化的结果norm不是0，才对磁力计数据进行更新计算
        {
            normR = 1.0f / norm;//三轴磁场归一化
            mx *= normR;
            my *= normR;
            mz *= normR;

            // compute reference direction of flux
					//计算参考方向
            hx = 2.0f * (mx * (0.5f - q2q2[1] - q3q3[1]) + my * (q1q2[1] - q0q3[1]) + mz * (q1q3[1] + q0q2[1]));

            hy = 2.0f * (mx * (q1q2[1] + q0q3[1]) + my * (0.5f - q1q1[1] - q3q3[1]) + mz * (q2q3[1] - q0q1[1]));

            hz = 2.0f * (mx * (q1q3[1] - q0q2[1]) + my * (q2q3[1] + q0q1[1]) + mz * (0.5f - q1q1[1] - q2q2[1]));

            bx = sqrt((hx * hx) + (hy * hy));

            bz = hz;

            // estimated direction of flux (w)
					//根据参考方向估计云台机体方向
            wx = 2.0f * (bx * (0.5f - q2q2[1] - q3q3[1]) + bz * (q1q3[1] - q0q2[1]));

            wy = 2.0f * (bx * (q1q2[1] - q0q3[1]) + bz * (q0q1[1] + q2q3[1]));

            wz = 2.0f * (bx * (q0q2[1] + q1q3[1]) + bz * (0.5f - q1q1[1] - q2q2[1]));

            exMag[1] = my * wz - mz * wy;//三轴磁场和估计方向进行叉积运算,计算估计方向与三轴磁场的偏差
            eyMag[1] = mz * wx - mx * wz;
            ezMag[1] = mx * wy - my * wx;

            // use un-extrapolated old values between magnetometer updates
            // dubious as dT does not apply to the magnetometer calculation so
            // time scaling is embedded in KpMag and KiMag
						//使用估计的旧值与磁力计值进行更新，dT不能应用在磁力计计算中，因此时间被嵌入在KpMag 和 KiMag里面
            gx += exMag[1] * eepromConfig.KpMag;//比例增益控制磁强计收敛速度
            gy += eyMag[1] * eepromConfig.KpMag;
            gz += ezMag[1] * eepromConfig.KpMag;

			
            if (eepromConfig.KiMag > 0.0f)//用积分增益控制陀螺仪的偏差收敛速率
            {
                exMagInt[1] += exMag[1] * eepromConfig.KiMag;
                eyMagInt[1] += eyMag[1] * eepromConfig.KiMag;
                ezMagInt[1] += ezMag[1] * eepromConfig.KiMag;

                gx += exMagInt[1];
                gy += eyMagInt[1];
                gz += ezMagInt[1];
            }
        }

        //-------------------------------------------

        // integrate quaternion rate
			 //四元数微分方程，其中halfT为测量周期，g为陀螺仪角速度，其余都是已知量，这里使用了一阶龙格库塔法求解四元数微分方程。
        q0i = (-q1[1] * gx - q2[1] * gy - q3[1] * gz) * halfT[1];
        q1i = (q0[1] * gx + q2[1] * gz - q3[1] * gy) * halfT[1];
        q2i = (q0[1] * gy - q1[1] * gz + q3[1] * gx) * halfT[1];
        q3i = (q0[1] * gz + q1[1] * gy - q2[1] * gx) * halfT[1];
        q0[1] += q0i;
        q1[1] += q1i;
        q2[1] += q2i;
        q3[1] += q3i;

        // normalise quaternion
				//四元数归一化，为什么又要归一化呢？这是因为引入了误差向量后四元数失去了规范性了(模不等于1了),所以要重新归一化
        normR = 1.0f / sqrt(q0[1] * q0[1] + q1[1] * q1[1] + q2[1] * q2[1] + q3[1] * q3[1]);
        q0[1] *= normR;
        q1[1] *= normR;
        q2[1] *= normR;
        q3[1] *= normR;

        // auxiliary variables to reduce number of repeated operations
				//把计算参考方向用到的值先都计算好,减少下面计算欧拉角时候的重复计算。
        q0q0[1] = q0[1] * q0[1];
        q0q1[1] = q0[1] * q1[1];
        q0q2[1] = q0[1] * q2[1];
        q0q3[1] = q0[1] * q3[1];
        q1q1[1] = q1[1] * q1[1];
        q1q2[1] = q1[1] * q2[1];
        q1q3[1] = q1[1] * q3[1];
        q2q2[1] = q2[1] * q2[1];
        q2q3[1] = q2[1] * q3[1];
        q3q3[1] = q3[1] * q3[1];

				//最后根据四元数方向余弦阵和欧拉角的转换关系，把四元数转换成欧拉角
        sensors[1].margAttitude500Hz[ROLL ] = atan2f(2.0f * (q0q1[1] + q2q3[1]), q0q0[1] - q1q1[1] - q2q2[1] + q3q3[1]);
        sensors[1].margAttitude500Hz[PITCH] = -asinf(2.0f * (q1q3[1] - q0q2[1]));
        sensors[1].margAttitude500Hz[YAW  ] = atan2f(2.0f * (q1q2[1] + q0q3[1]), q0q0[1] + q1q1[1] - q2q2[1] - q3q3[1]);

       cliPrintF("\r\n ROLL[1] %d ", (int)(sensors[1].margAttitude500Hz[ROLL]*100));
       cliPrintF(" PITCH[1] %d ", (int)(sensors[1].margAttitude500Hz[PITCH]*100));
       cliPrintF(" YAW[1] %d \r\n", (int)(sensors[1].margAttitude500Hz[YAW]*100));

    }
}

//====================================================================================================
// END OF CODE
//====================================================================================================
