/*
 * icm42688_calib.c
 *
 *  Created on: Jul 14, 2026
 *      Author: User
 *
 * Extended with Gyroscope calibration (bias correction)
 */


/*
 * icm42688_logger.c
 *
 * Implementation for ICM42688 data logging via UART
 * Prints CSV format data for Python serial logger to capture
 *
 *  Created on: Jul 12, 2026
 */

#include "icm42688_calib.h"
#include "icm42688.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ================================================================
   ACCELEROMETER CALIBRATION
   ================================================================ */

/* Offset vector ao, units = raw LSB */
static const float s_ao_accel[3] = {
    34.230000f,
   -109.057544f,
    197.146842f
};

/* Calibration matrix Cs_lsb (scale + misalignment), dimensionless */
static const float s_Cs_accel[3][3] = {
    { 0.999250f, 0.001926f,  0.030781f },
    { -0.004758f, 0.999405f,  -0.000741f },
    { -0.021634f,  0.002714f,  1.000719f }
};

/* ================================================================
   GYROSCOPE CALIBRATION
   ================================================================

   Method: Bias correction only
   - Gyroscope bias determined from static calibration
   - Future enhancement: scale factor + misalignment matrix

   Bias values (LSB):
     X: -2387.390217 LSB
     Y: -5033.908508 LSB
     Z:    94.005908 LSB

   ================================================================ */

/* Bias vector (gyroscope offset), units = raw LSB */
static const float s_bias_gyro[3] = {
   -2387.390217f,
   -5033.908508f,
     94.005908f
};

/* Scale factor matrix (identity for now, future: can add scale/misalignment) */
static const float s_Cs_gyro[3][3] = {
    { 1.0f, 0.0f, 0.0f },
    { 0.0f, 1.0f, 0.0f },
    { 0.0f, 0.0f, 1.0f }
};

/* ================================================================
   TIMING CONTROL (50 Hz = 20ms)
   ================================================================ */

static uint32_t last_time = 0;


/* ================================================================
   ACCELEROMETER CALIBRATION FUNCTIONS
   ================================================================ */

/**
 * icm42688_calib_apply_accel
 *
 * Apply accelerometer calibration to raw sensor data
 *
 * Formula: a_corrected = Cs @ (a_raw - ao)
 *
 * @param raw: pointer to raw ICM42688 data
 * @param out: pointer to calibrated output (g-units after scale conversion)
 */
void icm42688_calib_apply_accel(const icm42688_data *raw, icm42688_calib_data *out)
{
    if (raw == NULL || out == NULL)
    {
        return;
    }

    /* Center: raw - ao */
    const float dx = (float)raw->x_accel - s_ao_accel[0];
    const float dy = (float)raw->y_accel - s_ao_accel[1];
    const float dz = (float)raw->z_accel - s_ao_accel[2];

    /* a_corrected = Cs @ (raw - ao) */
    out->x_accel = s_Cs_accel[0][0] * dx + s_Cs_accel[0][1] * dy + s_Cs_accel[0][2] * dz;
    out->y_accel = s_Cs_accel[1][0] * dx + s_Cs_accel[1][1] * dy + s_Cs_accel[1][2] * dz;
    out->z_accel = s_Cs_accel[2][0] * dx + s_Cs_accel[2][1] * dy + s_Cs_accel[2][2] * dz;
}

/* ================================================================
   GYROSCOPE CALIBRATION FUNCTIONS
   ================================================================ */

/**
 * icm42688_calib_apply_gyro
 *
 * Apply gyroscope calibration to raw sensor data
 * Current method: Bias correction only
 *
 * Formula: w_corrected = Cs_gyro @ (w_raw - bias)
 *
 * @param raw: pointer to raw ICM42688 data
 * @param out: pointer to calibrated output (dps after scale conversion)
 */
void icm42688_calib_apply_gyro(const icm42688_data *raw, icm42688_calib_data *out)
{
    if (raw == NULL || out == NULL)
    {
        return;
    }

    /* Center: raw - bias */
    const float dwx = (float)raw->x_gyro - s_bias_gyro[0];
    const float dwy = (float)raw->y_gyro - s_bias_gyro[1];
    const float dwz = (float)raw->z_gyro - s_bias_gyro[2];

    /* w_corrected = Cs_gyro @ (raw - bias) */
    /* For now, Cs_gyro is identity, so this is just centering */
    out->x_gyro = s_Cs_gyro[0][0] * dwx + s_Cs_gyro[0][1] * dwy + s_Cs_gyro[0][2] * dwz;
    out->y_gyro = s_Cs_gyro[1][0] * dwx + s_Cs_gyro[1][1] * dwy + s_Cs_gyro[1][2] * dwz;
    out->z_gyro = s_Cs_gyro[2][0] * dwx + s_Cs_gyro[2][1] * dwy + s_Cs_gyro[2][2] * dwz;
}

/**
 * icm42688_calib_apply_accel_and_gyro
 *
 * Apply both accelerometer and gyroscope calibration
 *
 * @param raw: pointer to raw ICM42688 data
 * @param out: pointer to calibrated output
 */
void icm42688_calib_apply_accel_and_gyro(const icm42688_data *raw, icm42688_calib_data *out)
{
    if (raw == NULL || out == NULL)
    {
        return;
    }

    /* Apply accelerometer calibration */
    icm42688_calib_apply_accel(raw, out);

    /* Apply gyroscope calibration */
    const float dwx = (float)raw->x_gyro - s_bias_gyro[0];
    const float dwy = (float)raw->y_gyro - s_bias_gyro[1];
    const float dwz = (float)raw->z_gyro - s_bias_gyro[2];

    out->x_gyro = s_Cs_gyro[0][0] * dwx + s_Cs_gyro[0][1] * dwy + s_Cs_gyro[0][2] * dwz;
    out->y_gyro = s_Cs_gyro[1][0] * dwx + s_Cs_gyro[1][1] * dwy + s_Cs_gyro[1][2] * dwz;
    out->z_gyro = s_Cs_gyro[2][0] * dwx + s_Cs_gyro[2][1] * dwy + s_Cs_gyro[2][2] * dwz;
}

/* ================================================================
   READ CALIBRATED DATA (CONVENIENCE FUNCTIONS)
   ================================================================ */

/**
 * icm42688_readCalibratedAccel
 *
 * Read raw sensor and return calibrated accelerometer data
 *
 * @param out: pointer to calibrated accel output
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef icm42688_readCalibratedAccel(icm42688_calib_data *out)
{
    icm42688_data raw;
    HAL_StatusTypeDef status;

    if (out == NULL)
    {
        return HAL_ERROR;
    }

    status = icm42688_readRawSensor(&raw);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }

    icm42688_calib_apply_accel(&raw, out);
    return HAL_OK;
}

/**
 * icm42688_readCalibratedGyro
 *
 * Read raw sensor and return calibrated gyroscope data
 *
 * @param out: pointer to calibrated gyro output
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef icm42688_readCalibratedGyro(icm42688_calib_data *out)
{
    icm42688_data raw;
    HAL_StatusTypeDef status;

    if (out == NULL)
    {
        return HAL_ERROR;
    }

    status = icm42688_readRawSensor(&raw);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }

    icm42688_calib_apply_gyro(&raw, out);
    return HAL_OK;
}

/**
 * icm42688_readCalibrated
 *
 * Read raw sensor and return both calibrated accelerometer and gyroscope data
 *
 * @param out: pointer to calibrated output (both accel + gyro)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef icm42688_readCalibrated(icm42688_calib_data *out)
{
    icm42688_data raw;
    HAL_StatusTypeDef status;

    if (out == NULL)
    {
        return HAL_ERROR;
    }

    status = icm42688_readRawSensor(&raw);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }

    icm42688_calib_apply_accel_and_gyro(&raw, out);
    return HAL_OK;
}

/* ================================================================
   DEBUG PRINTING FUNCTIONS
   ================================================================ */

/**
 * icm42688_calib_print_accel
 *
 * Print raw vs calibrated accelerometer data
 */
void icm42688_calib_print_accel(void)
{
    icm42688_data raw;
    icm42688_calib_data cal;

    if (icm42688_readRawSensor(&raw) != HAL_OK)
    {
        printf("[ERROR] Failed to read ICM42688\r\n");
        return;
    }

    icm42688_calib_apply_accel(&raw, &cal);

    printf("ACCEL raw: %6d %6d %6d | cal: %8.3f %8.3f %8.3f\r\n",
           raw.x_accel, raw.y_accel, raw.z_accel,
           cal.x_accel, cal.y_accel, cal.z_accel);
}

/**
 * icm42688_calib_print_gyro
 *
 * Print raw vs calibrated gyroscope data
 */
void icm42688_calib_print_gyro(void)
{
    icm42688_data raw;
    icm42688_calib_data cal;

    if (icm42688_readRawSensor(&raw) != HAL_OK)
    {
        printf("[ERROR] Failed to read ICM42688\r\n");
        return;
    }

    icm42688_calib_apply_gyro(&raw, &cal);

    printf("GYRO raw:  %6d %6d %6d | cal: %8.3f %8.3f %8.3f\r\n",
           raw.x_gyro, raw.y_gyro, raw.z_gyro,
           cal.x_gyro, cal.y_gyro, cal.z_gyro);
}

/**
 * icm42688_calib_print
 *
 * Print raw vs calibrated accelerometer and gyroscope data
 */
void icm42688_calib_print(void)
{
    icm42688_data raw;
    icm42688_calib_data cal;

    if (icm42688_readRawSensor(&raw) != HAL_OK)
    {
        printf("[ERROR] Failed to read ICM42688\r\n");
        return;
    }

    icm42688_calib_apply_accel_and_gyro(&raw, &cal);

    printf("ACC raw: %6d %6d %6d | cal: %8.3f %8.3f %8.3f | "
           "GYR raw: %6d %6d %6d | cal: %8.3f %8.3f %8.3f\r\n",
           raw.x_accel, raw.y_accel, raw.z_accel,
           cal.x_accel, cal.y_accel, cal.z_accel,
           raw.x_gyro, raw.y_gyro, raw.z_gyro,
           cal.x_gyro, cal.y_gyro, cal.z_gyro);
}

/**
 * icm42688_calib_print_params
 *
 * Print calibration parameters for verification
 */
void icm42688_calib_print_params(void)
{
    printf("\n=== Accelerometer Calibration Parameters ===\r\n");
    printf("Offset vector ao (LSB):\r\n");
    printf("  X: %.6f\r\n", s_ao_accel[0]);
    printf("  Y: %.6f\r\n", s_ao_accel[1]);
    printf("  Z: %.6f\r\n", s_ao_accel[2]);

    printf("\nCalibration matrix Cs:\r\n");
    printf("  [%.6f  %.6f  %.6f]\r\n", s_Cs_accel[0][0], s_Cs_accel[0][1], s_Cs_accel[0][2]);
    printf("  [%.6f  %.6f  %.6f]\r\n", s_Cs_accel[1][0], s_Cs_accel[1][1], s_Cs_accel[1][2]);
    printf("  [%.6f  %.6f  %.6f]\r\n", s_Cs_accel[2][0], s_Cs_accel[2][1], s_Cs_accel[2][2]);

    printf("\n=== Gyroscope Calibration Parameters ===\r\n");
    printf("Bias vector (LSB):\r\n");
    printf("  X: %.6f\r\n", s_bias_gyro[0]);
    printf("  Y: %.6f\r\n", s_bias_gyro[1]);
    printf("  Z: %.6f\r\n", s_bias_gyro[2]);

    printf("\nScale matrix Cs_gyro:\r\n");
    printf("  [%.6f  %.6f  %.6f]\r\n", s_Cs_gyro[0][0], s_Cs_gyro[0][1], s_Cs_gyro[0][2]);
    printf("  [%.6f  %.6f  %.6f]\r\n", s_Cs_gyro[1][0], s_Cs_gyro[1][1], s_Cs_gyro[1][2]);
    printf("  [%.6f  %.6f  %.6f]\r\n", s_Cs_gyro[2][0], s_Cs_gyro[2][1], s_Cs_gyro[2][2]);
    printf("\n");
}

/* ================================================================
   LOGGING FUNCTIONS (50 Hz)
   ================================================================ */

/**
 * icm_logger_start
 *
 * Start logging and print CSV header
 */
void icm_logger_start(void)
{
    last_time = 0;

#if ICM_LOG_GYRO_ONLY
    printf("time,gx,gy,gz\r\n");
    printf("logging gyro only\r\n");

#elif ICM_LOG_ACCEL_ONLY
    printf("time,ax,ay,az\r\n");
    printf("logging accel only\r\n");

#elif ICM_LOG_BOTH
    printf("time,ax,ay,az,gx,gy,gz\r\n");
    printf("logging both accel and gyro\r\n");

#endif
}

/**
 * icm_logger_update
 *
 * Read sensor and log one sample (50 Hz = 20ms interval)
 */
void icm_logger_update(void)
{
    // Timing: 20ms = 50 Hz
    if (HAL_GetTick() - last_time < 20)
    {
        return;
    }

    last_time = HAL_GetTick();

    // Read sensor
    icm42688_data sensor_data;

    if (icm42688_readRawSensor(&sensor_data) != HAL_OK)
    {
        printf("[ERROR] Failed to read ICM42688\r\n");
        return;
    }

    // Print based on mode

#if ICM_LOG_GYRO_ONLY

    printf("%lu,%d,%d,%d\r\n",
           HAL_GetTick(),
           sensor_data.x_gyro,
           sensor_data.y_gyro,
           sensor_data.z_gyro);


#elif ICM_LOG_ACCEL_ONLY

    printf("%lu,%d,%d,%d\r\n",
           HAL_GetTick(),
           sensor_data.x_accel,
           sensor_data.y_accel,
           sensor_data.z_accel);

#elif ICM_LOG_BOTH

    printf("%lu,%d,%d,%d,%d,%d,%d\r\n",
           HAL_GetTick(),
           sensor_data.x_accel,
           sensor_data.y_accel,
           sensor_data.z_accel,
           sensor_data.x_gyro,
           sensor_data.y_gyro,
           sensor_data.z_gyro);


#endif
}

/* ================================================================
   CALIBRATED LOGGING FUNCTIONS (50 Hz)
   ================================================================

   Optional: Log calibrated data instead of raw

   ================================================================ */

/**
 * icm_logger_calibrated_start
 *
 * Start logging calibrated data and print CSV header
 */
void icm_logger_calibrated_start(void)
{
    last_time = 0;

#if ICM_LOG_GYRO_ONLY
    printf("time,gx_cal,gy_cal,gz_cal\r\n");
    printf("logging calibrated gyro only\r\n");

#elif ICM_LOG_ACCEL_ONLY
    printf("time,ax_cal,ay_cal,az_cal\r\n");
    printf("logging calibrated accel only\r\n");

#elif ICM_LOG_BOTH
    printf("time,ax_cal,ay_cal,az_cal,gx_cal,gy_cal,gz_cal\r\n");
    printf("logging calibrated accel and gyro\r\n");

#endif
}

/**
 * icm_logger_calibrated_update
 *
 * Read sensor, apply calibration, and log one sample (50 Hz = 20ms interval)
 */
void icm_logger_calibrated_update(void)
{
    // Timing: 20ms = 50 Hz
    if (HAL_GetTick() - last_time < 20)
    {
        return;
    }

    last_time = HAL_GetTick();

    // Read sensor
    icm42688_data raw;
    icm42688_calib_data calibrated;

    if (icm42688_readRawSensor(&raw) != HAL_OK)
    {
        printf("[ERROR] Failed to read ICM42688\r\n");
        return;
    }

    // Apply calibration
    icm42688_calib_apply_accel_and_gyro(&raw, &calibrated);

    // Print based on mode

#if ICM_LOG_GYRO_ONLY

    printf("%lu,%.3f,%.3f,%.3f\r\n",
           HAL_GetTick(),
           calibrated.x_gyro,
           calibrated.y_gyro,
           calibrated.z_gyro);


#elif ICM_LOG_ACCEL_ONLY

    printf("%lu,%.3f,%.3f,%.3f\r\n",
           HAL_GetTick(),
           calibrated.x_accel,
           calibrated.y_accel,
           calibrated.z_accel);

#elif ICM_LOG_BOTH

    printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           HAL_GetTick(),
           calibrated.x_accel,
           calibrated.y_accel,
           calibrated.z_accel,
           calibrated.x_gyro,
           calibrated.y_gyro,
           calibrated.z_gyro);


#endif
}

/* ================================================================
   USAGE IN main.c
   ================================================================

// Example 1: Simple raw logging
void main_logging_task(void)
{
    icm_logger_start();
    while (1)
    {
        icm_logger_update();
        HAL_Delay(10);
    }
}

// Example 2: Calibrated logging
void main_calibrated_logging_task(void)
{
    icm_logger_calibrated_start();
    while (1)
    {
        icm_logger_calibrated_update();
        HAL_Delay(10);
    }
}

// Example 3: Real-time calibrated reading with debug print
void main_real_time_task(void)
{
    icm42688_calib_print_params();  // Print params once at startup

    while (1)
    {
        icm42688_calib_print();  // Print both raw & calibrated
        HAL_Delay(100);  // 10 Hz for readability
    }
}

// Example 4: Calibrated accel + gyro separate
void main_separate_task(void)
{
    while (1)
    {
        icm42688_calib_print_accel();
        icm42688_calib_print_gyro();
        HAL_Delay(50);  // 20 Hz
    }
}

   ================================================================ */
