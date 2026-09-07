/*
 * quat2euler.c
 *
 *  Created on: Jun 30, 2026
 *      Author: User
 */

/*
 *  Implements the conversion of estimated quaternion (output of the EKF
 *  in attitude_kalman_extended.c) to Euler angles roll (phi), pitch
 *  (theta) and yaw (psi) using Eq. (15) from:
 *
 *  D.-J. Jwo, "Estimation of Quaternion Motion for GPS-Based Attitude
 *  Determination Using the Extended Kalman Filter," Computers,
 *  Materials & Continua, 2021, vol. 66, no. 2, pp. 2105-2126.
 *
 *      phi   = atan2( 2*(q1*q2 + q3*q4) , 1 - 2*(q2^2 + q3^2) )   -> roll
 *      theta = asin ( 2*(q1*q3 - q2*q4) )                          -> pitch
 *      psi   = atan2( 2*(q1*q4 + q2*q3) , 1 - 2*(q3^2 + q4^2) )   -> yaw
 *
 *  Mapping used in this project:
 *      q1 = q.s , q2 = q.x , q3 = q.y , q4 = q.z
 *
 *  Target: STM32L4R5ZIP, using CMSIS-DSP (arm_math.h) for sqrt-type
 *  helpers; atan2f/asinf from the standard math library are used for
 *  the inverse trigonometric functions, since they give correct
 *  quadrant resolution and are supported by the ARM Cortex-M4F FPU.
 */

#include "quat2euler.h"
#include "math.h"

#define RAD_TO_DEG_F  57.29577951308232f

static float32_t clamp_f32(float32_t val, float32_t min_val, float32_t max_val)
{
    if (val > max_val)
    {
        return max_val;
    }
    else if (val < min_val)
    {
        return min_val;
    }
    return val;
}

void quat2euler(quaternion q, uint8_t deg_unit, euler_angle *out)
{
    float32_t q1 = q.s;
    float32_t q2 = q.x;
    float32_t q3 = q.y;
    float32_t q4 = q.z;

    float32_t phi_num, phi_den;
    float32_t theta_arg;
    float32_t psi_num, psi_den;

    /* ---- Roll (phi) ---- */
    phi_num = 2.0f * (q1 * q2 + q3 * q4);
    phi_den = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    out->phi = atan2f(phi_num, phi_den);

    /* ---- Pitch (theta) ---- */
    theta_arg = 2.0f * (q1 * q3 - q2 * q4);
    theta_arg = clamp_f32(theta_arg, -1.0f, 1.0f); /* avoid NaN from asinf */
    out->theta = asinf(theta_arg);

    /* ---- Yaw (psi) ---- */
    psi_num = 2.0f * (q1 * q4 + q2 * q3);
    psi_den = 1.0f - 2.0f * (q3 * q3 + q4 * q4);
    out->psi = atan2f(psi_num, psi_den);

    if (deg_unit)
    {
        out->phi   *= RAD_TO_DEG_F;
        out->theta *= RAD_TO_DEG_F;
        out->psi   *= RAD_TO_DEG_F;
    }
}
