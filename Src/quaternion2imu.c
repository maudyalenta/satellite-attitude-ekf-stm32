/*
 * quaternion2imu.c
 *
 *  Created on: Jun 29, 2026
 *      Author: User
 */

#include "quaternion2imu.h"
/**
 * @brief This function uses accelerometer/magnetomter readings to compute frame rotation
 * quaternions
 * @param qa quaternion
 * @param mag_hor magnetomer x-axis data in the reference frame
 * @param mag_ver magnetomer z-axis data in the reference frame
 * @param imu_data normalized imu data (prediction)
 */
void quater2imu(quaternion qa, imu_norm *imu_data, float32_t mag_hor, float32_t mag_ver)
{
	imu_data ->ax = 2 * (qa.x * qa.z - qa.s * qa.y);
	imu_data ->ay = 2 * (qa.y * qa.z + qa.s * qa.x);
	imu_data ->az = qa.s * qa.s - qa.x * qa.x - qa.y * qa.y + qa.z * qa.z;

	imu_data -> mx = (qa.s * qa.s + qa.x * qa.x - qa.y * qa.y - qa.z * qa.z) * mag_hor
			+ (2 * (qa.x * qa.z - qa.s * qa.y)) * mag_ver;
	imu_data -> my = (2 * (qa.x * qa.y - qa.s * qa.z)) * mag_hor
			+ (2 * (qa.y * qa.z + qa.s * qa.x)) * mag_ver;
	imu_data -> mz = (2 * (qa.x * qa.z + qa.s * qa.y)) * mag_hor
			+ (qa.s * qa.s - qa.x * qa.x - qa.y * qa.y + qa.z * qa.z) * mag_ver;
}
