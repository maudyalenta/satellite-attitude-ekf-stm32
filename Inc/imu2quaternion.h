/*
 * imu2quaternion.h
 *
 *  Created on: Jun 29, 2026
 *      Author: User
 */

#ifndef INC_IMU2QUATERNION_H_
#define INC_IMU2QUATERNION_H_

#include "imu_norm.h"
#include "quaternion.h"
#include "arm_math.h"

void accmag2quat(quaternion *qtotal, quaternion *qacc, quaternion *qmag,
		imu_norm imu_data, float32_t *lx, float32_t *ly);
void mag2quat(quaternion *qmag, imu_norm imu_data, float32_t lx, float32_t ly);
void quat_gyro(quaternion *qa, imu_norm imu_data);
void acc2quat(quaternion *qa, imu_norm imu_data);
void mag2quat(quaternion *qmag, imu_norm imu_data, float32_t lx, float32_t ly);


#endif /* INC_IMU2QUATERNION_H_ */
