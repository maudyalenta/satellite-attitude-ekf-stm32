/*
 * icm42688_calib.h
 *
 *  Created on: Jul 14, 2026
 *      Author: User
 *
 * Header file for ICM42688 sensor calibration functions
 * Supports both accelerometer and gyroscope calibration
 */

#ifndef ICM42688_CALIB_H_
#define ICM42688_CALIB_H_

#include "stm32l4xx_hal.h"
#include "icm42688.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
   STRUCTURES
   ================================================================ */

/* Raw sensor data structure (defined in icm42688.h)
typedef struct {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
} icm42688_data;
*/

/* Calibrated sensor data structure */
typedef struct {
    float x_accel;      // calibrated acceleration (g-units after scale)
    float y_accel;
    float z_accel;
    float x_gyro;       // calibrated angular velocity (dps after scale)
    float y_gyro;
    float z_gyro;
} icm42688_calib_data;

/* ================================================================
   ACCELEROMETER CALIBRATION FUNCTIONS
   ================================================================ */

/**
 * icm42688_calib_apply_accel
 *
 * Apply accelerometer calibration to raw sensor data
 *
 * @param raw: pointer to raw ICM42688 data (input)
 * @param out: pointer to calibrated output (output)
 */
void icm42688_calib_apply_accel(const icm42688_data *raw, icm42688_calib_data *out);

/**
 * icm42688_readCalibratedAccel
 *
 * Read raw sensor and return calibrated accelerometer data
 *
 * @param out: pointer to calibrated accel output
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef icm42688_readCalibratedAccel(icm42688_calib_data *out);

/* ================================================================
   GYROSCOPE CALIBRATION FUNCTIONS
   ================================================================ */

/**
 * icm42688_calib_apply_gyro
 *
 * Apply gyroscope calibration to raw sensor data
 * Current method: Bias correction only
 *
 * @param raw: pointer to raw ICM42688 data (input)
 * @param out: pointer to calibrated output (output)
 */
void icm42688_calib_apply_gyro(const icm42688_data *raw, icm42688_calib_data *out);

/**
 * icm42688_readCalibratedGyro
 *
 * Read raw sensor and return calibrated gyroscope data
 *
 * @param out: pointer to calibrated gyro output
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef icm42688_readCalibratedGyro(icm42688_calib_data *out);

/* ================================================================
   COMBINED CALIBRATION FUNCTIONS
   ================================================================ */

/**
 * icm42688_calib_apply_accel_and_gyro
 *
 * Apply both accelerometer and gyroscope calibration
 *
 * @param raw: pointer to raw ICM42688 data (input)
 * @param out: pointer to calibrated output (output)
 */
void icm42688_calib_apply_accel_and_gyro(const icm42688_data *raw, icm42688_calib_data *out);

/**
 * icm42688_readCalibrated
 *
 * Read raw sensor and return both calibrated accelerometer and gyroscope data
 *
 * @param out: pointer to calibrated output (both accel + gyro)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef icm42688_readCalibrated(icm42688_calib_data *out);

/* ================================================================
   DEBUG PRINTING FUNCTIONS
   ================================================================ */

/**
 * icm42688_calib_print_accel
 *
 * Print raw vs calibrated accelerometer data
 */
void icm42688_calib_print_accel(void);

/**
 * icm42688_calib_print_gyro
 *
 * Print raw vs calibrated gyroscope data
 */
void icm42688_calib_print_gyro(void);

/**
 * icm42688_calib_print
 *
 * Print raw vs calibrated accelerometer and gyroscope data
 */
void icm42688_calib_print(void);

/**
 * icm42688_calib_print_params
 *
 * Print all calibration parameters for verification
 */
void icm42688_calib_print_params(void);

/* ================================================================
   LOGGING FUNCTIONS (Raw Data - 50 Hz)
   ================================================================ */

/**
 * icm_logger_start
 *
 * Start logging and print CSV header for raw data
 */
void icm_logger_start(void);

/**
 * icm_logger_update
 *
 * Read sensor and log one sample of raw data (50 Hz = 20ms interval)
 */
void icm_logger_update(void);

/* ================================================================
   CALIBRATED LOGGING FUNCTIONS (Calibrated Data - 50 Hz)
   ================================================================ */

/**
 * icm_logger_calibrated_start
 *
 * Start logging and print CSV header for calibrated data
 */
void icm_logger_calibrated_start(void);

/**
 * icm_logger_calibrated_update
 *
 * Read sensor, apply calibration, and log one sample (50 Hz = 20ms interval)
 */
void icm_logger_calibrated_update(void);

/* ================================================================
   USAGE EXAMPLES
   ================================================================

// Example 1: Read calibrated accel + gyro once
void example_single_read(void)
{
    icm42688_calib_data cal;
    if (icm42688_readCalibrated(&cal) == HAL_OK)
    {
        printf("Accel: %.3f, %.3f, %.3f g\r\n", cal.x_accel, cal.y_accel, cal.z_accel);
        printf("Gyro:  %.3f, %.3f, %.3f dps\r\n", cal.x_gyro, cal.y_gyro, cal.z_gyro);
    }
}

// Example 2: Real-time debug output
void main_debug_task(void)
{
    icm42688_calib_print_params();  // Print params once at startup

    while (1)
    {
        icm42688_calib_print();  // Print both raw & calibrated
        HAL_Delay(100);  // 10 Hz for readability
    }
}

// Example 3: Log calibrated data to UART
void main_calibrated_logging_task(void)
{
    icm_logger_calibrated_start();  // Print header

    while (1)
    {
        icm_logger_calibrated_update();  // Log at 50 Hz
        HAL_Delay(10);
    }
}

// Example 4: Accel-only reading
void example_accel_only(void)
{
    icm42688_calib_data cal;
    if (icm42688_readCalibratedAccel(&cal) == HAL_OK)
    {
        printf("Accel: %.3f, %.3f, %.3f g\r\n", cal.x_accel, cal.y_accel, cal.z_accel);
    }
}

// Example 5: Gyro-only reading
void example_gyro_only(void)
{
    icm42688_calib_data cal;
    if (icm42688_readCalibratedGyro(&cal) == HAL_OK)
    {
        printf("Gyro: %.3f, %.3f, %.3f dps\r\n", cal.x_gyro, cal.y_gyro, cal.z_gyro);
    }
}

   ================================================================ */

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_CALIB_H_ */
