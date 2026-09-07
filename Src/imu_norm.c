/*
 * imu_norm.c
 *
 *  Created on: May 30, 2026
 *      Author: User
 */


#include "imu_norm.h"
#include "icm42688_calib.h"
#include <stdio.h>

void sensor2imu(icm42688_calib_data icm_data, float mx,
	    float my,
	    float mz,imu_norm* imu)
{
    float32_t norm_acc;
    float32_t norm_mag;

    arm_status status;

    status = arm_sqrt_f32(
        ((float32_t)icm_data.x_accel * icm_data.x_accel) +
        ((float32_t)icm_data.y_accel * icm_data.y_accel) +
        ((float32_t)icm_data.z_accel * icm_data.z_accel),
        &norm_acc
    );


    if(status != ARM_MATH_SUCCESS)
    {
        printf("ACC sqrt error\r\n");
        while(1);
    }


    status = arm_sqrt_f32(
           mx*mx +
           my*my +
           mz*mz,
           &norm_mag);

       if(status != ARM_MATH_SUCCESS)
       {
           printf("MAG sqrt error\r\n");
           while(1);
       }

    /* NORMALIZED ACCEL */
    imu->ax = ((float32_t)icm_data.x_accel) / norm_acc;
    imu->ay = ((float32_t)icm_data.y_accel) / norm_acc;
    imu->az = ((float32_t)icm_data.z_accel) / norm_acc;


    /* GYRO rad/s */
    imu->gx = ((float32_t)icm_data.x_gyro) * GYRO_SCALE;
    imu->gy = ((float32_t)icm_data.y_gyro) * GYRO_SCALE;
    imu->gz = ((float32_t)icm_data.z_gyro) * GYRO_SCALE;


    /* Normalized magnetometer */

     imu->mx = mx / norm_mag;
     imu->my = my / norm_mag;
     imu->mz = mz / norm_mag;
}
