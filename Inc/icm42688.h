/*
 * icm42688.h
 *
 *  Created on: Mar 7, 2026
 *      Author: Maudy
 */

/*#ifndef INC_ICM_42688_H_
#define INC_ICM_42688_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"

typedef struct{
	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
} icm42688_data;



#define ICM42688_I2C_ADDR           0x68

#define MREG_ADDR 				0x7C
#define MREG_DATA 				0x7D

#define GYRO_RANGE_VALUE        _gyro_15_625dps
#define ACCEL_RANGE_VALUE       _accel_2g


#define REG_BANK_SEL     0x76
#define WHO_AM_I         0x75
#define WHO_AM_I_VALUE   0x47
#define PWR_MGMT_0       0x4E

#define GYRO_CONFIG_0    0x4F
#define GYRO_CONFIG_1    0x51

#define ACCEL_CONFIG_0   0x50
#define ACCEL_CONFIG_1   0x53

#define ACCEL_DATA_X1    0x1F
#define ACCEL_DATA_X0    0x20
#define ACCEL_DATA_Y1    0x21
#define ACCEL_DATA_Y0    0x22
#define ACCEL_DATA_Z1    0x23
#define ACCEL_DATA_Z0    0x24

#define GYRO_DATA_X1     0x25
#define GYRO_DATA_X0     0x26
#define GYRO_DATA_Y1     0x27
#define GYRO_DATA_Y0     0x28
#define GYRO_DATA_Z1     0x29
#define GYRO_DATA_Z0     0x2A


#define OFFSET_USER0       0x77
#define OFFSET_USER1       0x78
#define OFFSET_USER2       0x79
#define OFFSET_USER3       0x7A
#define OFFSET_USER4       0x7B
#define OFFSET_USER5       0x7C
#define OFFSET_USER6       0x7D
#define OFFSET_USER7       0x7E
#define OFFSET_USER8       0x7F

typedef enum
{
	_gyro_2000dps 	= 0x00,
	_gyro_1000dps 	= 0x01,
	_gyro_500dps  	= 0x02,
	_gyro_250dps  	= 0x03,
	_gyro_125dps 	= 0x04,
	_gyro_62_5dps 	= 0x05,
	_gyro_31_25dps 	= 0x06,
	_gyro_15_625dps = 0x07

} gyro_range;

typedef enum
{
	_accel_16g = 0x00,
	_accel_8g  = 0x01,
	_accel_4g  = 0x02,
	_accel_2g  = 0x03
} accel_range;

typedef enum
{
	_b0 = 0,
	_b1 = 1 << 4,
	_b2 = 2 << 4,
	_b3 = 3 << 4,
	_b4 = 4 << 4,
}user_bank;

HAL_StatusTypeDef icm42688_select_bank(I2C_HandleTypeDef *hi2c, user_bank bank);
HAL_StatusTypeDef icm42688_check_connection(I2C_HandleTypeDef *hi2c);
uint8_t icm42688_read_whoami(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef icm42688_wakeup(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef icm42688_set_accel(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef icm42688_set_gyro(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef icm42688_read_sensor(I2C_HandleTypeDef *hi2c,icm42688_data *imu_data);
HAL_StatusTypeDef icm42688_calibrate_gyro(I2C_HandleTypeDef *hi2c);


#endif /* INC_ICM_42688_H_ */



/*
 * icm42688.h
 *
 *  Created on: Jun 30, 2026
 *      Author: User
 *
 *  ICM-42688-P driver header.
 *  Architecture mirrors the BNO055 driver pattern:
 *    - icm42688.h / icm42688.c   : device logic, no HAL calls
 *    - icm42688_stm32.h          : HAL transport (writeReg / readReg)
 *
 *  Usage:
 *    1. Include icm42688_stm32.h BEFORE icm42688.h in your main/init file
 *       so the transport functions are resolved.
 *    2. Call icm42688_assignI2C(&hi2c2) first, then icm42688_setup().
 */

#ifndef INC_ICM42688_H_
#define INC_ICM42688_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"

/* =========================================================
 * I2C Address
 * ========================================================= */
#define ICM42688_I2C_ADDR_LO    0x68        /* AD0 pin = GND */
#define ICM42688_I2C_ADDR_HI    0x69        /* AD0 pin = VCC */
#define ICM42688_I2C_ADDR       ICM42688_I2C_ADDR_LO

/* =========================================================
 * WHO_AM_I
 * ========================================================= */
#define ICM42688_ID             0x47

/* =========================================================
 * Register Map — Bank 0 (default)
 * ========================================================= */
#define ICM42688_REG_BANK_SEL       0x76

#define ICM42688_WHO_AM_I           0x75
#define ICM42688_PWR_MGMT_0         0x4E

#define ICM42688_GYRO_CONFIG_0      0x4F    /* FS & ODR */
#define ICM42688_GYRO_CONFIG_1      0x51    /* filter */
#define ICM42688_ACCEL_CONFIG_0     0x50    /* FS & ODR */
#define ICM42688_ACCEL_CONFIG_1     0x53    /* filter */

/* Raw data registers (big-endian, MSB first) */
#define ICM42688_ACCEL_DATA_X1      0x1F
#define ICM42688_GYRO_DATA_X1       0x25

/* =========================================================
 * Register Map — Bank 4 (gyro / accel offset)
 * ========================================================= */
#define ICM42688_OFFSET_USER0       0x77
#define ICM42688_OFFSET_USER1       0x78
#define ICM42688_OFFSET_USER2       0x79
#define ICM42688_OFFSET_USER3       0x7A
#define ICM42688_OFFSET_USER4       0x7B

/* =========================================================
 * PWR_MGMT_0 bit fields
 * ========================================================= */
#define ICM42688_PWR_ACCEL_LN       (0x03)  /* accel low-noise */
#define ICM42688_PWR_GYRO_LN        (0x0C)  /* gyro  low-noise */
#define ICM42688_PWR_ALL_ON         (ICM42688_PWR_ACCEL_LN | ICM42688_PWR_GYRO_LN)

/* =========================================================
 * Gyro Full-Scale (GYRO_CONFIG_0 bits [7:5])
 * ========================================================= */
typedef enum
{
    ICM42688_GYRO_FS_2000DPS    = (0x00 << 5),
    ICM42688_GYRO_FS_1000DPS    = (0x01 << 5),
    ICM42688_GYRO_FS_500DPS     = (0x02 << 5),
    ICM42688_GYRO_FS_250DPS     = (0x03 << 5),
    ICM42688_GYRO_FS_125DPS     = (0x04 << 5),
    ICM42688_GYRO_FS_62_5DPS    = (0x05 << 5),
    ICM42688_GYRO_FS_31_25DPS   = (0x06 << 5),
    ICM42688_GYRO_FS_15_625DPS  = (0x07 << 5),
} icm42688_gyro_fs_t;

/* =========================================================
 * Gyro ODR (GYRO_CONFIG_0 bits [3:0])
 * ========================================================= */
typedef enum
{
    ICM42688_GYRO_ODR_8KHZ      = 0x03,
    ICM42688_GYRO_ODR_4KHZ      = 0x04,
    ICM42688_GYRO_ODR_2KHZ      = 0x05,
    ICM42688_GYRO_ODR_1KHZ      = 0x06,
    ICM42688_GYRO_ODR_200HZ     = 0x07,
    ICM42688_GYRO_ODR_100HZ     = 0x08,
    ICM42688_GYRO_ODR_50HZ      = 0x09,
} icm42688_gyro_odr_t;

/* =========================================================
 * Accel Full-Scale (ACCEL_CONFIG_0 bits [7:5])
 * ========================================================= */
typedef enum
{
    ICM42688_ACCEL_FS_16G       = (0x00 << 5),
    ICM42688_ACCEL_FS_8G        = (0x01 << 5),
    ICM42688_ACCEL_FS_4G        = (0x02 << 5),
    ICM42688_ACCEL_FS_2G        = (0x03 << 5),
} icm42688_accel_fs_t;

/* =========================================================
 * Accel ODR (ACCEL_CONFIG_0 bits [3:0])
 * ========================================================= */
typedef enum
{
    ICM42688_ACCEL_ODR_8KHZ     = 0x03,
    ICM42688_ACCEL_ODR_4KHZ     = 0x04,
    ICM42688_ACCEL_ODR_2KHZ     = 0x05,
    ICM42688_ACCEL_ODR_1KHZ     = 0x06,
    ICM42688_ACCEL_ODR_200HZ    = 0x07,
    ICM42688_ACCEL_ODR_100HZ    = 0x08,
    ICM42688_ACCEL_ODR_50HZ     = 0x09,
} icm42688_accel_odr_t;

/* =========================================================
 * User bank select
 * ========================================================= */
typedef enum
{
    ICM42688_BANK_0 = 0x00,
    ICM42688_BANK_1 = 0x01,
    ICM42688_BANK_2 = 0x02,
    ICM42688_BANK_3 = 0x03,
    ICM42688_BANK_4 = 0x04,
} icm42688_bank_t;

/* =========================================================
 * Configuration struct (passed once to icm42688_setup)
 * ========================================================= */
typedef struct
{
    icm42688_gyro_fs_t  gyro_fs;
    icm42688_gyro_odr_t gyro_odr;
    icm42688_accel_fs_t accel_fs;
    icm42688_accel_odr_t accel_odr;
} icm42688_config_t;

/* Default config matching your original 0x6E / 0xE6 register values:
 *   ACCEL: FS=2g, ODR=1kHz   (0x6E = [6:5]=11 => 2g, [3:0]=0110 => 1kHz)
 *   GYRO : FS=15.625dps, ODR=1kHz (0xE6 = [7:5]=111 => 15.625, [3:0]=0110 => 1kHz) */
#define ICM42688_CONFIG_DEFAULT  { \
    .gyro_fs   = ICM42688_GYRO_FS_15_625DPS, \
    .gyro_odr  = ICM42688_GYRO_ODR_1KHZ,    \
    .accel_fs  = ICM42688_ACCEL_FS_2G,      \
    .accel_odr = ICM42688_ACCEL_ODR_1KHZ,   \
}

/* =========================================================
 * Raw sensor data (before calibration / scaling)
 * ========================================================= */
typedef struct
{
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
} icm42688_data;

/* =========================================================
 * Transport functions — implemented in icm42688_stm32.h
 * ========================================================= */
void             icm42688_assignI2C(I2C_HandleTypeDef *hi2c_device);
HAL_StatusTypeDef icm42688_writeReg(uint8_t reg, uint8_t data);
HAL_StatusTypeDef icm42688_readReg(uint8_t reg, uint8_t *data, uint8_t len);
void             icm42688_delay(uint32_t ms);

/* =========================================================
 * Public API
 * ========================================================= */

/* Initialisation */
HAL_StatusTypeDef icm42688_setup(icm42688_config_t cfg);

/* Identity & connection checks */
HAL_StatusTypeDef icm42688_checkConnection(void);
uint8_t           icm42688_readWhoAmI(void);

/* Individual config (called internally by setup, but also
 * available if the user wants to reconfigure at runtime)  */
HAL_StatusTypeDef icm42688_selectBank(icm42688_bank_t bank);
HAL_StatusTypeDef icm42688_wakeup(void);
HAL_StatusTypeDef icm42688_configAccel(icm42688_accel_fs_t fs, icm42688_accel_odr_t odr);
HAL_StatusTypeDef icm42688_configGyro(icm42688_gyro_fs_t  fs, icm42688_gyro_odr_t  odr);

/* Data read */
HAL_StatusTypeDef icm42688_readRawSensor(icm42688_data *out);

/* Gyro bias (applied automatically inside readRawSensor) */
void icm42688_setGyroBias(int16_t bx, int16_t by, int16_t bz);
void icm42688_getGyroBias(int16_t *bx, int16_t *by, int16_t *bz);

#ifdef __cplusplus
}
#endif

#endif /* INC_ICM42688_H_ */

