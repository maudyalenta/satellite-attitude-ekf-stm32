/*
 * attitude_kalman_extended.h
 *
 *  Created on: Jun 29, 2026
 *      Author: User
 */

#ifndef INC_ATTITUDE_KALMAN_EXTENDED_H_
#define INC_ATTITUDE_KALMAN_EXTENDED_H_


#include "arm_math.h"
#include "imu_norm.h"
#include "imu2quaternion.h"
#include "quaternion.h"

void attitude_init_extended();
void run_kalman_extended(imu_norm imu, quaternion *qout);
void print_matrix(arm_matrix_instance_f32 matrix);


#endif /* INC_ATTITUDE_KALMAN_EXTENDED_H_ */
