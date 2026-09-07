/*
 * imu_norm.h
 *
 *  Created on: May 30, 2026
 *      Author: User
 */

#ifndef INC_IMU_NORM_H_
#define INC_IMU_NORM_H_


#include "main.h"
#include "icm42688.h"
#include "icm42688_calib.h"
#include "arm_math.h"
#include <math.h>

#define SAMPLE_RATE   1000.0f
#define GYRO_FS_DPS 15.625f
//#define PI 3.14159265359f

/* rad/sample */
#define GYRO_SCALE \
((GYRO_FS_DPS * PI) / (180.0f * 32768.0f)) / SAMPLE_RATE

#define GYRO_SCALE_DPS \
(GYRO_FS_DPS / 32768.0f)

typedef struct
{
    float32_t ax;
    float32_t ay;
    float32_t az;

    float32_t gx;
    float32_t gy;
    float32_t gz;

    float32_t mx;
    float32_t my;
    float32_t mz;


} imu_norm;

//void sensor2imu(icm42688_data icm_data, imu_norm* imu);
void sensor2imu(
	icm42688_calib_data icm_data,
    float mx,
    float my,
    float mz,
    imu_norm *imu);



#endif /* INC_IMU_NORM_H_ */
