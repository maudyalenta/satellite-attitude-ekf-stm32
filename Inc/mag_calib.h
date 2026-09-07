/*
 * mag_calib.h
 *
 *  Created on: Jun 29, 2026
 *      Author: User
 */

#ifndef INC_MAG_CALIB_H_
#define INC_MAG_CALIB_H_

#include <stdint.h>
#include "bno055.h"

// Hard-Iron Bias (DC offset)
#define MAG_BIAS_X    (-57.3514f)
#define MAG_BIAS_Y    (24.5094f)
#define MAG_BIAS_Z    (-460.6202f)


// Soft-Iron Correction Matrix (3×3)
#define MAG_A00       (0.9814f)
#define MAG_A01       (-0.0075f)
#define MAG_A02       (-0.0167f)

#define MAG_A10       (-0.0075f)
#define MAG_A11       (1.0171f)
#define MAG_A12       (0.0128f)

#define MAG_A20       (-0.0167f)
#define MAG_A21       (0.0128f)
#define MAG_A22       (1.0024f)


typedef struct
{
    // Old interface (for compatibility with existing code)
    float xmean, ymean, zmean;
    float xrange, yrange, zrange;

    // New interface (ellipsoid fitting)
    float bias[3];              // Hard-iron bias [bx, by, bz]
    float soft_iron[3][3];      // Soft-iron correction matrix [3×3]
    bool calibration_valid;     // Flag: calibration loaded

} mag_calib_t;

void mag_calib_init(mag_calib_t *cal);

void mag_calibrate(mag_calib_t *cal);

void mag_correct_ellipsoid(
    int16_t mx_raw,
    int16_t my_raw,
    int16_t mz_raw,
    const mag_calib_t *cal,
    float *mx,
    float *my,
    float *mz);


void mag_correct(
    int16_t mx_raw,
    int16_t my_raw,
    int16_t mz_raw,
    mag_calib_t *cal,
    float *mx,
    float *my,
    float *mz);

float mag_get_magnitude(float mx, float my, float mz);

void mag_print_coefficients(void);

void mag_print_status(const mag_calib_t *cal);
void mag_calib_print(const mag_calib_t *cal);


void mag_logger_start(void);
void mag_logger_update(void);

#endif /* INC_MAG_CALIB_H_ */
