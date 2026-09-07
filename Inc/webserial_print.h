/*
 * webserial_print.h
 *
 *  Created on: Jun 30, 2026
 *      Author: User
 */

#ifndef INC_WEBSERIAL_PRINT_H_
#define INC_WEBSERIAL_PRINT_H_


#include "arm_math.h"
#include "quaternion2imu.h"  /* quaternion struct: s, x, y, z */
#include "quat2euler.h"      /* euler_angle struct: phi, theta, psi */

/*
 * webserial_print_frame()
 * -----------------------------------------------------------------------
 * Prints orientation (degrees), quaternion, and static calibration data
 * via printf() in the format required by the Adafruit Web Serial 3D
 * model viewer.
 *
 * Parameters:
 *  euler : orientation in degrees (phi=roll, theta=pitch, psi=yaw),
 *          must be obtained with quat2euler(q, 1, &euler) -> deg_unit=1
 *  quat  : the same quaternion used to compute 'euler' (q.s=w, q.x, q.y, q.z)
 */
void webserial_print_frame(euler_angle euler, quaternion quat);


#endif /* INC_WEBSERIAL_PRINT_H_ */
