/*
 * quat2euler.h
 *
 *  Created on: Jun 30, 2026
 *      Author: User
 */

#ifndef INC_QUAT2EULER_H_
#define INC_QUAT2EULER_H_

/*
 * quat2euler.h
 *
 *  Created on: Jun 30, 2026
 *      Author: User
 *
 *  Conversion of unit quaternion (q1,q2,q3,q4) to Euler angles
 *  (roll phi, pitch theta, yaw psi) using Eq. (15) from:
 *  D.-J. Jwo, "Estimation of Quaternion Motion for GPS-Based
 *  Attitude Determination Using the Extended Kalman Filter,"
 *  CMC, 2021, vol. 66, no. 2, pp. 2105-2126.
 *
 *  Quaternion convention used in this project (matches
 *  attitude_kalman_extended.c):
 *      q1 = q.s   (scalar part)
 *      q2 = q.x
 *      q3 = q.y
 *      q4 = q.z
 *
 *  Target: STM32L4R5ZIP (CMSIS DSP / arm_math.h available)
 */

#include "arm_math.h"
#include "quaternion2imu.h"   /* provides the 'quaternion' struct: s, x, y, z */


/* Struct to hold the resulting Euler angles */
typedef struct
{
    float32_t phi;     /* roll  (bank angle),  rotation about body X axis  */
    float32_t theta;   /* pitch (pitch angle), rotation about body Y axis  */
    float32_t psi;      /* yaw   (heading angle), rotation about body Z axis */
} euler_angle;

/*
 * quat2euler()
 * -----------------------------------------------------------------------
 * Converts a unit quaternion to Euler angles (roll, pitch, yaw) according
 * to Eq. (15) of the referenced paper.
 *
 *      phi   = atan2( 2*(q1*q2 + q3*q4) , 1 - 2*(q2^2 + q3^2) )
 *      theta = asin ( 2*(q1*q3 - q2*q4) )
 *      psi   = atan2( 2*(q1*q4 + q2*q3) , 1 - 2*(q3^2 + q4^2) )
 *
 * Notes:
 *  - atan2 is used instead of plain atan/tan^-1 so that the correct
 *    quadrant is resolved automatically (avoids +-180 deg ambiguity).
 *  - The argument of asin() is clamped to [-1, 1] to avoid NaN results
 *    caused by floating point rounding errors (quaternion not perfectly
 *    normalized).
 *
 * Parameters:
 *  q        : input quaternion (q.s=q1, q.x=q2, q.y=q3, q.z=q4)
 *  deg_unit : 1 -> output angles in degrees, 0 -> output angles in radians
 *  out      : pointer to euler_angle struct where the result is stored
 */
void quat2euler(quaternion q, uint8_t deg_unit, euler_angle *out);


#endif /* INC_QUAT2EULER_H_ */
