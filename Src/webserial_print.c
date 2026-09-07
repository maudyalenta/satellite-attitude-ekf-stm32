/*
 * webserial_print.c
 *
 *  Created on: Jun 30, 2026
 *      Author: User
 */


/*
 *  Sends orientation, quaternion, and (static) calibration data through
 *  the serial port in the exact text format expected by the Adafruit
 *  3D Model Viewer (Web Serial) tool:
 *
 *      Orientation: <heading>, <roll>, <pitch>\r\n
 *      Quaternion: <w>, <x>, <y>, <z>\r\n
 *      Calibration: <sys>, <gyro>, <accel>, <mag>\r\n
 *
 *  Data source:
 *   - euler_angle (phi=roll, theta=pitch, psi=yaw)  from quat2euler()
 *   - quaternion  (s=q1/w, x=q2, y=q3, z=q4)        from run_kalman_extended()
 *
 *  Mapping note:
 *   Adafruit's original BNO055 example sends euler.x as heading,
 *   euler.y as roll, euler.z as pitch (BNO055's own euler convention),
 *   and uses (360 - heading) so the 3D model viewer rotates in the
 *   expected on-screen direction. To stay compatible with that viewer,
 *   our yaw (psi) is mapped to "heading", our roll (phi) to "roll",
 *   and our pitch (theta) to "pitch".
 */

#include "webserial_print.h"
#include "stdio.h"

void webserial_print_frame(euler_angle euler, quaternion quat)
{
    float32_t heading = euler.psi;   /* yaw   -> heading */
    float32_t roll    = euler.phi;   /* roll  -> roll    */
    float32_t pitch   = euler.theta; /* pitch -> pitch   */

    /* normalize heading to 0..360 range before flipping it,
     * same convention as the Adafruit example (360 - x) */
    if (heading < 0.0f)
    {
        heading += 360.0f;
    }

    printf(
        "Orientation: %.2f, %.2f, %.2f\r\n",
        360.0f - heading,
        roll,
        pitch);

    printf(
        "Quaternion: %.4f, %.4f, %.4f, %.4f\r\n",
        quat.s,
        quat.x,
        quat.y,
        quat.z);

    /* Calibration data: not available from this EKF pipeline,
     * so it is reported as fully calibrated (3,3,3,3), the
     * maximum value expected by the Adafruit Web Serial viewer. */
    printf(
        "Calibration: %d, %d, %d, %d\r\n",
        3,
        3,
        3,
        3);
}
