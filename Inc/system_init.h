/*
 * system_init.h
 *
 *  Created on: Jun 30, 2026
 *      Author: User
 *
 *  Single entry-point for all sensor hardware initialisation:
 *    1. ICM-42688-P  (accel + gyro) via I2C
 *    2. BNO055       (magnetometer only, raw) via I2C
 *
 *  Call sequence in main.c:
 *    system_sensors_init(&hi2c2);          // setup + WHO_AM_I check
 *    system_sensors_calibrate();           // gyro bias calibration
 *    system_sensors_set_mode(SENSOR_MODE_FIFO);  // or DIRECT
 */

#ifndef INC_SYSTEM_INIT_H_
#define INC_SYSTEM_INIT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/* =========================================================
 * Sensor read mode
 * ========================================================= */
//typedef enum
//{
//    SENSOR_MODE_DIRECT = 0,   /* ICM: read registers directly (polling) */
//    SENSOR_MODE_FIFO   = 1,   /* ICM: read via FIFO Packet 3            */
//} sensor_mode_t;

/* =========================================================
 * Init status
 * ========================================================= */
typedef enum
{
    SYS_INIT_OK            = 0,
    SYS_INIT_ERR_ICM_CONN  = 1,   /* ICM42688 I2C not responding         */
    SYS_INIT_ERR_ICM_ID    = 2,   /* ICM42688 WHO_AM_I mismatch          */
    SYS_INIT_ERR_ICM_SETUP = 3,   /* ICM42688 config register write fail */
    SYS_INIT_ERR_BNO_CONN  = 4,   /* BNO055 I2C not responding           */
    SYS_INIT_ERR_BNO_ID    = 5,   /* BNO055 CHIP_ID mismatch             */
} system_init_status_t;

/* =========================================================
 * Public API
 * ========================================================= */

/*
 * system_sensors_init()
 * -----------------------------------------------------------------------
 * Full hardware init sequence for ICM42688 and BNO055:
 *   ICM42688:
 *     - I2C device ready check
 *     - WHO_AM_I verify (0x47)
 *     - Soft reset
 *     - Wakeup (accel + gyro low-noise mode)
 *     - Set ODR and FS (Accel: ±2g / 1kHz, Gyro: ±15.625dps / 1kHz)
 *   BNO055:
 *     - I2C device ready check
 *     - CHIP_ID verify (0xA0)
 *     - Reset + config mode
 *     - Set operation mode to AMG (raw accel+gyro+mag, no fusion)
 *       → used only for raw magnetometer in this project
 *
 * Parameters:
 *   hi2c : I2C handle shared by both sensors
 *
 * Returns SYS_INIT_OK or a specific error code.
 * Prints detailed diagnostics via printf (UART).
 */
system_init_status_t system_sensors_init(I2C_HandleTypeDef *hi2c);

/*
 * system_sensors_calibrate()
 * -----------------------------------------------------------------------
 * Run gyro bias calibration (ICM42688 only).
 * Sensor must be stationary during this call (~1 second).
 * Result is stored internally and applied to every subsequent read.
 *
 * Note: magnetometer hard-iron calibration (mag_calibrate) is
 * intentionally kept separate — it requires rotation and takes ~20 s.
 * Call mag_calibrate() separately before starting the EKF.
 */
void system_sensors_calibrate(void);

/*
 * system_sensors_set_mode()
 * -----------------------------------------------------------------------
 * Switch ICM42688 between FIFO (Packet 3) and direct register read.
 *   SENSOR_MODE_FIFO   : enables FIFO, configures timestamp + temp
 *   SENSOR_MODE_DIRECT : disables FIFO, use icm42688_readRawSensor()
 */
//void system_sensors_set_mode(sensor_mode_t mode);

/*
 * system_sensors_get_mode()
 * Return the currently active read mode.
 */
//sensor_mode_t system_sensors_get_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_SYSTEM_INIT_H_ */
