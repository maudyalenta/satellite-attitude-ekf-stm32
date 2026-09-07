/*
 * sensor_manager.c
 *
 *  Created on: Jun 6, 2026
 *      Author: User
 */

#include "sensor_manager.h"

sensor_data_t sensor;

void SensorManager_Init(void)
{
    bno055_setup();

    HAL_Delay(100);

    bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}


void SensorManager_Update(void)
{
    bno055_vector_t acc;
    bno055_vector_t gyro;
    bno055_vector_t mag;
    bno055_vector_t euler;
    bno055_vector_t quat;

    acc   = bno055_getVectorAccelerometer();
    gyro  = bno055_getVectorGyroscope();
    mag   = bno055_getVectorMagnetometer();
    euler = bno055_getVectorEuler();
    quat  = bno055_getVectorQuaternion();

    sensor.bno_ax = acc.x;
    sensor.bno_ay = acc.y;
    sensor.bno_az = acc.z;

    sensor.bno_gx = gyro.x;
    sensor.bno_gy = gyro.y;
    sensor.bno_gz = gyro.z;

    sensor.bno_mx = mag.x;
    sensor.bno_my = mag.y;
    sensor.bno_mz = mag.z;

    sensor.bno_yaw   = euler.x;
    sensor.bno_pitch = euler.y;
    sensor.bno_roll  = euler.z;

    sensor.bno_qw = quat.w;
    sensor.bno_qx = quat.x;
    sensor.bno_qy = quat.y;
    sensor.bno_qz = quat.z;
}
