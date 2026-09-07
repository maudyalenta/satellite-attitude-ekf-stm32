/*
 * imu2quaternion.c
 *
 *  Created on: Jun 29, 2026
 *      Author: User
 */

#include "imu2quaternion.h"
#include "arm_math.h"
#include "stdio.h"








//static imu_norm flip_z_axis(imu_norm imu_data)
//{
//    imu_norm imu_fixed = imu_data;
//
//    // Keep X and Y, flip Z
//    imu_fixed.ax = imu_data.ax;      // Keep X accel
//    imu_fixed.ay = imu_data.ay;      // Keep Y accel
//    imu_fixed.az = -imu_data.az;     // Flip Z accel (Z+ up → Z+ down)
//
//    // For magnetometer (if needed)
//    imu_fixed.mx = imu_data.mx;      // Keep X mag
//    imu_fixed.my = imu_data.my;      // Keep Y mag
//    imu_fixed.mz = -imu_data.mz;     // Flip Z mag
//
//    // For gyroscope (if needed)
//    imu_fixed.gx = imu_data.gx;      // Keep X gyro
//    imu_fixed.gy = imu_data.gy;      // Keep Y gyro
//    imu_fixed.gz = -imu_data.gz;     // Flip Z gyro
//
//    return imu_fixed;
//}

/**
 * @brief This function uses accelerometer/magnetomter readings to compute frame rotation
 * quaternions
 * @param qtotal final quaternion
 * @param qacc quaternion resulting from accelerometer data
 * @param qmag quaternion resulting from compass data
 * @param imu_data normalized imu data
 */
void accmag2quat(quaternion *qtotal, quaternion *qacc, quaternion *qmag,
		imu_norm imu_data, float32_t *lx, float32_t* ly)
{
	//imu_norm imu_fixed = flip_z_axis(imu_data);

//	qmag->s = 0;
//	qmag->x = imu_fixed.mx;  // Use fixed magnetometer
//	qmag->y = imu_fixed.my;
//	qmag->z = imu_fixed.mz;

	qmag->s = 0;  qmag->x = imu_data.mx; qmag->y = imu_data.my; qmag->z = imu_data.mz;
	acc2quat(qacc, imu_data);
	vector_rotate(*qacc, *qmag, qmag);
	*lx = qmag -> x;
	*ly = qmag -> y;
	mag2quat(qmag, imu_data, *lx, *ly);

	quat_mult(qtotal, *qmag, *qacc);
}

/**
 * @brief This function uses accelerometer readings to compute frame rotation
 * quaternion.
 * @param qa quaternion
 * @param imu_data normalized imu data
 */
void acc2quat(quaternion *qa, imu_norm imu_data)
{
	arm_status arm_status_temp;
	float32_t lambda;
	if(imu_data.az >= 0)
	{
		arm_status_temp = arm_sqrt_f32(((imu_data.az + 1)/2), &lambda);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
			printf("error sqrt! %d \n", arm_status_temp);
			while(1);
		}
		qa -> s = lambda;
		qa -> x = imu_data.ay / (2 * lambda);
		qa -> y = -imu_data.ax / (2 * lambda);
		qa -> z = 0;
	}
	else
	{
		arm_status_temp = arm_sqrt_f32((1 - imu_data.az) / 2, &lambda);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
			printf("error sqrt! %d \n", arm_status_temp);
			while(1);
		}
		qa -> s = -imu_data.ay / (2 * lambda);
		qa -> x = -lambda;
		qa -> y =  0 ;
		qa -> z = -imu_data.ax / (2 * lambda);
	}

}

/**
 * @brief This function uses magnetometer readings to compute frame rotation
 * quaternion.
 * @param qmag quaternion that defines rotation alon z-axis
 * @param imu_data normalized imu data
 */
void mag2quat(quaternion *qmag, imu_norm imu_data, float32_t lx, float32_t ly)
{
	arm_status arm_status_temp;
	float32_t gamma, gamma_sqrt;

	gamma = lx * lx + ly * ly;
	// gamma_sqrt is Mx
	arm_status_temp = arm_sqrt_f32(gamma, &gamma_sqrt);
	if(arm_status_temp != ARM_MATH_SUCCESS)
	{
		printf("error sqrt! %d \n", arm_status_temp);
		while(1);
	}
	if(lx >= 0)
	{
		arm_status_temp = arm_sqrt_f32(0.5 + lx / ( 2 * gamma_sqrt), &(qmag -> s));
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
			printf("error sqrt! %d \n", arm_status_temp);
			while(1);
		}
		qmag -> x = 0;
		qmag -> y = 0;
		qmag -> z = -ly / (qmag -> s * 2 * gamma_sqrt);
	}
	else
	{
		arm_status_temp = arm_sqrt_f32(0.5 - lx / ( 2 * gamma_sqrt), &(qmag -> z));
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
			printf("error sqrt! %d \n", arm_status_temp);
			while(1);
		}
		qmag -> x = 0;
		qmag -> y = 0;
		qmag -> s = -ly / (qmag ->z * 2 * gamma_sqrt);

	}
}

/**
 * @brief This function uses gyroscope data to update the quaternion
 * @param qa quaternion
 * @param imu_data normalized imu data
 */
void quat_gyro(quaternion *qa, imu_norm imu_data)
{
	quaternion quat_temp = *qa;
	qa ->s += (- quat_temp.x * imu_data.gx - quat_temp.y * imu_data.gy
			- quat_temp.z * imu_data.gz) / 2;
	qa ->x += (quat_temp.s * imu_data.gx + quat_temp.y * imu_data.gz -
			quat_temp.z * imu_data.gy) / 2;
	qa ->y += (quat_temp.s * imu_data.gy - quat_temp.x * imu_data.gz +
			quat_temp.z * imu_data.gx) / 2;
	qa ->z += (quat_temp.s * imu_data.gz + quat_temp.x * imu_data.gy -
			quat_temp.y * imu_data.gx) / 2;
}
