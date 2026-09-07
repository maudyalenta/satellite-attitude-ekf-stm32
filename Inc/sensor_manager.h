/*
 * sensor_manager.h
 *
 *  Created on: Jun 6, 2026
 *      Author: User
 */

#ifndef INC_SENSOR_MANAGER_H_
#define INC_SENSOR_MANAGER_H_


#include "bno055.h"
#include "icm42688.h"

typedef struct
{
    /* BNO055 */

    float bno_ax;
    float bno_ay;
    float bno_az;

    float bno_gx;
    float bno_gy;
    float bno_gz;

    float bno_mx;
    float bno_my;
    float bno_mz;

    float bno_roll;
    float bno_pitch;
    float bno_yaw;

    float bno_qw;
    float bno_qx;
    float bno_qy;
    float bno_qz;

} sensor_data_t;

extern sensor_data_t sensor;

void SensorManager_Init(void);
void SensorManager_Update(void);


#endif /* INC_SENSOR_MANAGER_H_ */
