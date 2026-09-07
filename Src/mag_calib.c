/*
 * mag_calib.c
 *
 * Magnetometer Calibration Implementation
 * Uses ellipsoid fitting (hard-iron bias + soft-iron correction matrix)
 *
 * Calibration from Python mag_cal.py:
 * - Hard-iron bias computed from ellipsoid center
 * - Soft-iron matrix computed from eigenvalue decomposition
 *
 *  Created on: Jun 29, 2026
 *  Updated: Jul 12, 2026
 */

#include "mag_calib.h"
#include "bno055.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <math.h>

/* ================================================================
   INITIALIZATION
   ================================================================ */

void mag_calib_init(mag_calib_t *cal)
{
    if (cal == NULL) return;

    // Load hard-iron bias
    cal->bias[0] = MAG_BIAS_X;
    cal->bias[1] = MAG_BIAS_Y;
    cal->bias[2] = MAG_BIAS_Z;

    // Load soft-iron matrix
    cal->soft_iron[0][0] = MAG_A00;
    cal->soft_iron[0][1] = MAG_A01;
    cal->soft_iron[0][2] = MAG_A02;

    cal->soft_iron[1][0] = MAG_A10;
    cal->soft_iron[1][1] = MAG_A11;
    cal->soft_iron[1][2] = MAG_A12;

    cal->soft_iron[2][0] = MAG_A20;
    cal->soft_iron[2][1] = MAG_A21;
    cal->soft_iron[2][2] = MAG_A22;

    cal->calibration_valid = true;

    printf("✓ Magnetometer calibration initialized\r\n");
    mag_print_coefficients();
}

/* ================================================================
   CALIBRATION APPLICATION - ELLIPSOID FITTING (NEW & CORRECT)
   ================================================================ */

void mag_correct_ellipsoid(
    int16_t mx_raw,
    int16_t my_raw,
    int16_t mz_raw,
    const mag_calib_t *cal,
    float *mx,
    float *my,
    float *mz)
{
    if (cal == NULL || mx == NULL || my == NULL || mz == NULL) {
        return;
    }

    // TAMBAHKAN INI
//        printf("DEBUG cal->bias = %.4f %.4f %.4f\r\n", cal->bias[0], cal->bias[1], cal->bias[2]);
//        printf("DEBUG soft_iron = [%.4f %.4f %.4f] [%.4f %.4f %.4f] [%.4f %.4f %.4f]\r\n",
//            cal->soft_iron[0][0], cal->soft_iron[0][1], cal->soft_iron[0][2],
//            cal->soft_iron[1][0], cal->soft_iron[1][1], cal->soft_iron[1][2],
//            cal->soft_iron[2][0], cal->soft_iron[2][1], cal->soft_iron[2][2]);

    // Step 1: Remove hard-iron bias (center the ellipsoid at origin)
    float centered[3];
    centered[0] = (float)mx_raw - cal->bias[0];
    centered[1] = (float)my_raw - cal->bias[1];
    centered[2] = (float)mz_raw - cal->bias[2];

    // Step 2: Apply soft-iron correction matrix
    // corrected = A_soft × centered
    //
    // [mx]   [A00  A01  A02] [centered_x]
    // [my] = [A10  A11  A12] [centered_y]
    // [mz]   [A20  A21  A22] [centered_z]

    *mx = cal->soft_iron[0][0] * centered[0] +
          cal->soft_iron[0][1] * centered[1] +
          cal->soft_iron[0][2] * centered[2];

    *my = cal->soft_iron[1][0] * centered[0] +
          cal->soft_iron[1][1] * centered[1] +
          cal->soft_iron[1][2] * centered[2];

    *mz = cal->soft_iron[2][0] * centered[0] +
          cal->soft_iron[2][1] * centered[1] +
          cal->soft_iron[2][2] * centered[2];
}


void mag_calib_print(const mag_calib_t *cal)
{
    int16_t mx_raw, my_raw, mz_raw;
    float mx_cal, my_cal, mz_cal;

    if (cal == NULL)
    {
        printf("[ERROR] Magnetometer calibration not initialized\r\n");
        return;
    }

    /* Read raw magnetometer */
    bno055_getRawMagnetometer(&mx_raw, &my_raw, &mz_raw);

    /* Apply calibration */
    mag_correct_ellipsoid(mx_raw,
                          my_raw,
                          mz_raw,
                          cal,
                          &mx_cal,
                          &my_cal,
                          &mz_cal);

    printf("MAG raw: %6d %6d %6d | cal: %8.3f %8.3f %8.3f\r\n",
           mx_raw,
           my_raw,
           mz_raw,
           mx_cal,
           my_cal,
           mz_cal);
}

/* ================================================================
   LEGACY FUNCTIONS (for backward compatibility)
   ================================================================ */

/**
 * Legacy function - uses simple min/max scaling
 * Kept for backward compatibility, but mag_correct_ellipsoid is better
 */
void mag_calibrate(mag_calib_t *cal)
{
    int16_t mx, my, mz;

    int16_t xmin = 32767;
    int16_t xmax = -32768;
    int16_t ymin = 32767;
    int16_t ymax = -32768;
    int16_t zmin = 32767;
    int16_t zmax = -32768;

    int32_t xsum = 0;
    int32_t ysum = 0;
    int32_t zsum = 0;

    printf("Rotate sensor for calibration\r\n");

    for(int i = 0; i < 1000; i++)
    {
        bno055_getRawMagnetometer(&mx, &my, &mz);

        if(mx < xmin) xmin = mx;
        if(mx > xmax) xmax = mx;

        if(my < ymin) ymin = my;
        if(my > ymax) ymax = my;

        if(mz < zmin) zmin = mz;
        if(mz > zmax) zmax = mz;

        xsum += mx;
        ysum += my;
        zsum += mz;

        HAL_Delay(20);
    }

    cal->xmean = (float)xsum / 1000.0f;
    cal->ymean = (float)ysum / 1000.0f;
    cal->zmean = (float)zsum / 1000.0f;

    cal->xrange = xmax - xmin;
    cal->yrange = ymax - ymin;
    cal->zrange = zmax - zmin;
}

/**
 * Legacy function - uses simple min/max scaling
 * Kept for backward compatibility, but mag_correct_ellipsoid is better
 */
void mag_correct(
    int16_t mx_raw,
    int16_t my_raw,
    int16_t mz_raw,
    mag_calib_t *cal,
    float *mx,
    float *my,
    float *mz)
{
    *mx = 2.0f * (mx_raw - cal->xmean) / cal->xrange;
    *my = 2.0f * (my_raw - cal->ymean) / cal->yrange;
    *mz = 2.0f * (mz_raw - cal->zmean) / cal->zrange;
}

/* ================================================================
   UTILITY FUNCTIONS
   ================================================================ */

float mag_get_magnitude(float mx, float my, float mz)
{
    return sqrtf(mx*mx + my*my + mz*mz);
}

void mag_print_coefficients(void)
{
    printf("\n");
    printf("================================================\r\n");
    printf("MAGNETOMETER CALIBRATION COEFFICIENTS\r\n");
    printf("================================================\r\n");

    printf("\nHard-Iron Bias (from ellipsoid center):\r\n");
    printf("  Bx = %.6f\r\n", MAG_BIAS_X);
    printf("  By = %.6f\r\n", MAG_BIAS_Y);
    printf("  Bz = %.6f\r\n", MAG_BIAS_Z);

    printf("\nSoft-Iron Correction Matrix (3×3):\r\n");
    printf("  [%.6f  %.6f  %.6f]\r\n", MAG_A00, MAG_A01, MAG_A02);
    printf("  [%.6f  %.6f  %.6f]\r\n", MAG_A10, MAG_A11, MAG_A12);
    printf("  [%.6f  %.6f  %.6f]\r\n", MAG_A20, MAG_A21, MAG_A22);

    printf("================================================\r\n\n");
}

void mag_print_status(const mag_calib_t *cal)
{
    if (cal == NULL) {
        printf("Calibration: NOT INITIALIZED\r\n");
        return;
    }

    printf("\n--- Magnetometer Calibration Status ---\r\n");
    printf("Status: %s\r\n", cal->calibration_valid ? "VALID" : "INVALID");
    printf("\nBias (hard-iron offset):\r\n");
    printf("  X: %.2f, Y: %.2f, Z: %.2f\r\n",
           cal->bias[0], cal->bias[1], cal->bias[2]);
    printf("\nSoft-iron diagonal:\r\n");
    printf("  A[0][0]: %.6f, A[1][1]: %.6f, A[2][2]: %.6f\r\n",
           cal->soft_iron[0][0], cal->soft_iron[1][1], cal->soft_iron[2][2]);
    printf("\n");
}

/* ================================================================
   LOGGER FUNCTIONS
   ================================================================ */

static uint32_t last_time = 0;

void mag_logger_start(void)
{
    printf("time,mx,my,mz\r\n");
}

void mag_logger_update(void)
{
    if (HAL_GetTick() - last_time >= 20)   // 50 Hz
    {
        last_time = HAL_GetTick();

        int16_t mx, my, mz;

        bno055_getRawMagnetometer(&mx, &my, &mz);

        printf("%lu,%d,%d,%d\r\n",
               HAL_GetTick(),
               mx,
               my,
               mz);
    }
}

/* ================================================================
   EXAMPLE USAGE IN YOUR APPLICATION
   ================================================================

#include "mag_calib.h"
#include "bno055.h"

// Global calibration structure
mag_calib_t mag_calib;

void main_init(void)
{
    // Initialize calibration with pre-computed coefficients
    mag_calib_init(&mag_calib);

    // Print status for verification
    mag_print_status(&mag_calib);
}

void process_magnetometer(void)
{
    int16_t mx_raw, my_raw, mz_raw;
    float mx_cal, my_cal, mz_cal;

    // Read raw data from BNO055
    bno055_getRawMagnetometer(&mx_raw, &my_raw, &mz_raw);

    // Apply ellipsoid fitting calibration (NEW - CORRECT)
    mag_correct_ellipsoid(mx_raw, my_raw, mz_raw, &mag_calib,
                          &mx_cal, &my_cal, &mz_cal);

    // Get field magnitude (should be ~49 µT for your location)
    float magnitude = mag_get_magnitude(mx_cal, my_cal, mz_cal);

    // Use calibrated values in attitude determination
    // ...
}

   ================================================================ */
