///*
//
//#include "icm42688.h"
//#include <stdio.h>
//
////// HELPER
////static HAL_StatusTypeDef icm42688_write_reg(I2C_HandleTypeDef *hi2c,uint8_t reg,uint8_t data)
////{
////    return HAL_I2C_Mem_Write(hi2c,IMU_ADDR,reg,I2C_MEMADD_SIZE_8BIT,&data,1,100);
////}
////
////static HAL_StatusTypeDef icm42688_select_bank(I2C_HandleTypeDef *hi2c,uint8_t bank)
////{
////    return icm42688_write_reg(hi2c,REG_BANK_SEL,bank);
////}
//
//
//static int16_t gyro_bias_x = 0,
//               gyro_bias_y = 0,
//               gyro_bias_z = 0;
//
//
///* ================= BANK SELECT ================= */
//HAL_StatusTypeDef icm42688_select_bank(I2C_HandleTypeDef *hi2c, user_bank bank)
//{
//    return HAL_I2C_Mem_Write(
//        hi2c,
//        IMU_ADDR,
//        REG_BANK_SEL,
//        I2C_MEMADD_SIZE_8BIT,
//        (uint8_t*)&bank,
//        1,
//        100
//    );
//}
//// CEK I2C
//HAL_StatusTypeDef icm42688_check_connection(I2C_HandleTypeDef *hi2c)
//{
//    return HAL_I2C_IsDeviceReady(
//        hi2c,
//		IMU_ADDR,
//        3,
//        100
//    );
//}
//
//// CEK SENSOR
//uint8_t icm42688_read_whoami(I2C_HandleTypeDef *hi2c)
//{
//    uint8_t whoami = 0;
//
//    if(HAL_I2C_Mem_Read(
//            hi2c,
//			IMU_ADDR,
//            WHO_AM_I,
//            I2C_MEMADD_SIZE_8BIT,
//            &whoami,
//            1,
//            100
//       ) != HAL_OK)
//    {
//        return 0xFF;
//    }
//
//    return whoami;
//}
//
//// WAKEUP SENSOR
//HAL_StatusTypeDef icm42688_wakeup(I2C_HandleTypeDef *hi2c)
//{
//    uint8_t data = 0x0F;
//
//    if(HAL_I2C_Mem_Write(
//            hi2c,
//			IMU_ADDR,
//			PWR_MGMT_0,
//            I2C_MEMADD_SIZE_8BIT,
//            &data,
//            1,
//            100
//       ) != HAL_OK)
//    {
//        return HAL_ERROR;
//    }
//
//    HAL_Delay(50);
//
//    return HAL_OK;
//}
//
//// CONFIG ACC
//HAL_StatusTypeDef icm42688_set_accel(I2C_HandleTypeDef *hi2c)
//{
//    uint8_t data = 0x6E;
//    uint8_t check;
//
//    if(HAL_I2C_Mem_Write(
//            hi2c,
//            IMU_ADDR,
//            ACCEL_CONFIG_0,
//            I2C_MEMADD_SIZE_8BIT,
//            &data,
//            1,
//            100
//       ) != HAL_OK)
//    {
//        return HAL_ERROR;
//    }
//
//    HAL_Delay(10);
//
//    if(HAL_I2C_Mem_Read(
//            hi2c,
//           IMU_ADDR,
//            ACCEL_CONFIG_0,
//            I2C_MEMADD_SIZE_8BIT,
//            &check,
//            1,
//            100
//       ) != HAL_OK)
//    {
//        return HAL_ERROR;
//    }
//
//    printf("ACCEL_CONFIG_0 = 0x%02X\n", check);
//
//    return HAL_OK;
//}
//
//
//// CONFIG GYRO
//HAL_StatusTypeDef icm42688_set_gyro(I2C_HandleTypeDef *hi2c)
//{
//    uint8_t data = 0xE6;
//    uint8_t check;
//
//    if(HAL_I2C_Mem_Write(
//            hi2c,
//            IMU_ADDR,
//            GYRO_CONFIG_0,
//            I2C_MEMADD_SIZE_8BIT,
//            &data,
//            1,
//            100
//       ) != HAL_OK)
//    {
//        return HAL_ERROR;
//    }
//
//    HAL_Delay(10);
//
//    if(HAL_I2C_Mem_Read(
//            hi2c,
//            IMU_ADDR,
//            GYRO_CONFIG_0,
//            I2C_MEMADD_SIZE_8BIT,
//            &check,
//            1,
//            100
//       ) != HAL_OK)
//    {
//        return HAL_ERROR;
//    }
//
//    printf("GYRO_CONFIG_0 = 0x%02X\n", check);
//
//    return HAL_OK;
//}
//
//
//// READ RAW DATA
//HAL_StatusTypeDef icm42688_read_sensor(
//    I2C_HandleTypeDef *hi2c,
//    icm42688_data *imu_data
//)
//{
//    uint8_t buf[12];
//
//    HAL_StatusTypeDef status;
//
//    status = HAL_I2C_Mem_Read(
//        hi2c,
//       IMU_ADDR,
//        ACCEL_DATA_X1,
//        I2C_MEMADD_SIZE_8BIT,
//        buf,
//        12,
//        100
//    );
//
//    if(status != HAL_OK)
//    {
//        return HAL_ERROR;
//    }
//
//    imu_data->x_accel = (int16_t)((buf[0] << 8) | buf[1]);
//    imu_data->y_accel = (int16_t)((buf[2] << 8) | buf[3]);
//    imu_data->z_accel = (int16_t)((buf[4] << 8) | buf[5]);
//
//    imu_data->x_gyro = (int16_t)((buf[6] << 8) | buf[7]);
//    imu_data->y_gyro = (int16_t)((buf[8] << 8) | buf[9]);
//    imu_data->z_gyro = (int16_t)((buf[10] << 8) | buf[11]);
//
//
//    imu_data->x_gyro -= gyro_bias_x;
//    imu_data->y_gyro -= gyro_bias_y;
//    imu_data->z_gyro -= gyro_bias_z;
//
//    return HAL_OK;
//}
//
//
//HAL_StatusTypeDef icm42688_calibrate_gyro(I2C_HandleTypeDef *hi2c)
//{
//    icm42688_data data;
//
//    int32_t x_sum = 0, y_sum = 0, z_sum = 0;
//
//
//    for(int i = 0; i < 500; i++)
//    {
//        if(icm42688_read_sensor(hi2c, &data) != HAL_OK)
//        {
//            return HAL_ERROR;
//        }
//
//        x_sum += data.x_gyro;
//        y_sum += data.y_gyro;
//        z_sum += data.z_gyro;
//
//        HAL_Delay(2);
//    }
//
//    gyro_bias_x = (x_sum / 500);
//    gyro_bias_y = (y_sum / 500);
//    gyro_bias_z = (z_sum / 500);
//
//    printf("Gyro Bias: X=%d Y=%d Z=%d\r\n",
//    		gyro_bias_x, gyro_bias_y, gyro_bias_z);
//
////    /* =========================
////       SWITCH TO BANK 4
////       ========================= */
////    icm42688_select_bank(hi2c, _b4);
////
////    /* =========================
////       WRITE OFFSET REGISTERS
////       ========================= */
////
////    icm42688_write_reg(hi2c,OFFSET_USER0,(uint8_t)(x_bias & 0xFF));
////    icm42688_write_reg(hi2c,OFFSET_USER1,((x_bias >> 8) & 0x0F) | ((y_bias >> 8) & 0x0F) << 4);
////    icm42688_write_reg(hi2c,OFFSET_USER2,(uint8_t)(y_bias & 0xFF));
////    icm42688_write_reg(hi2c,OFFSET_USER3,(z_bias >> 8) & 0x0F);
////
////
////    /* =========================
////       BACK TO BANK 0
////       ========================= */
////    icm42688_select_bank(hi2c, _b0);
////
////    printf("Gyro offset written to hardware\r\n");
//
//    return HAL_OK;
//}

/*
 * icm42688.c
 *
 *  Created on: Jun 30, 2026
 *      Author: User
 *
 *  ICM-42688-P device logic.
 *  All hardware access goes through icm42688_writeReg() and
 *  icm42688_readReg(), which are implemented in icm42688_stm32.h.
 *  This file never calls HAL_I2C_* directly.
 */

#include "icm42688.h"
#include <stdio.h>

/* =========================================================
 * Private: gyro bias (applied in icm42688_readRawSensor)
 * ========================================================= */
static int16_t s_gyro_bias_x = 0;
static int16_t s_gyro_bias_y = 0;
static int16_t s_gyro_bias_z = 0;

/* =========================================================
 * icm42688_selectBank()
 * Switch register bank (0-4).
 * Always return to Bank 0 after accessing Bank 4.
 * ========================================================= */
HAL_StatusTypeDef icm42688_selectBank(icm42688_bank_t bank)
{
    HAL_StatusTypeDef status;

    status = icm42688_writeReg(ICM42688_REG_BANK_SEL, (uint8_t)bank);

    if (status != HAL_OK)
    {
        printf("[ICM42688] selectBank(%d) failed\r\n", bank);
    }

    return status;
}

/* =========================================================
 * icm42688_checkConnection()
 * ========================================================= */
HAL_StatusTypeDef icm42688_checkConnection(void)
{
    uint8_t id = icm42688_readWhoAmI();

    if (id == ICM42688_ID)
    {
        return HAL_OK;
    }

    printf("[ICM42688] checkConnection FAILED: WHO_AM_I=0x%02X (expected 0x%02X)\r\n",
           id, ICM42688_ID);

    return HAL_ERROR;
}

/* =========================================================
 * icm42688_readWhoAmI()
 * ========================================================= */
uint8_t icm42688_readWhoAmI(void)
{
    uint8_t id = 0xFF;
    icm42688_readReg(ICM42688_WHO_AM_I, &id, 1);
    return id;
}

/* =========================================================
 * icm42688_wakeup()
 * Enable accel + gyro in low-noise mode.
 * After writing PWR_MGMT_0, datasheet requires >=200 µs
 * before the first valid read; we wait 1 ms to be safe.
 * ========================================================= */
HAL_StatusTypeDef icm42688_wakeup(void)
{
    HAL_StatusTypeDef status;

    status = icm42688_writeReg(ICM42688_PWR_MGMT_0, ICM42688_PWR_ALL_ON);

    if (status != HAL_OK)
    {
        printf("[ICM42688] wakeup failed\r\n");
        return HAL_ERROR;
    }

    icm42688_delay(1);   /* ≥ 200 µs per datasheet, 1 ms margin */

    return HAL_OK;
}

/* =========================================================
 * icm42688_configAccel()
 * Set full-scale and ODR, then read-back to verify.
 * ========================================================= */
HAL_StatusTypeDef icm42688_configAccel(icm42688_accel_fs_t fs,
                                        icm42688_accel_odr_t odr)
{
    uint8_t reg_val = (uint8_t)fs | (uint8_t)odr;
    uint8_t readback = 0;
    HAL_StatusTypeDef status;

    status = icm42688_writeReg(ICM42688_ACCEL_CONFIG_0, reg_val);
    if (status != HAL_OK)
    {
        printf("[ICM42688] configAccel write failed\r\n");
        return HAL_ERROR;
    }

    icm42688_delay(1);

    status = icm42688_readReg(ICM42688_ACCEL_CONFIG_0, &readback, 1);
    if (status != HAL_OK)
    {
        printf("[ICM42688] configAccel readback failed\r\n");
        return HAL_ERROR;
    }

    if (readback != reg_val)
    {
        printf("[ICM42688] configAccel mismatch: wrote 0x%02X read 0x%02X\r\n",
               reg_val, readback);
        return HAL_ERROR;
    }

    printf("[ICM42688] ACCEL_CONFIG_0 = 0x%02X OK\r\n", readback);
    return HAL_OK;
}

/* =========================================================
 * icm42688_configGyro()
 * Set full-scale and ODR, then read-back to verify.
 * ========================================================= */
HAL_StatusTypeDef icm42688_configGyro(icm42688_gyro_fs_t  fs,
                                       icm42688_gyro_odr_t odr)
{
    uint8_t reg_val = (uint8_t)fs | (uint8_t)odr;
    uint8_t readback = 0;
    HAL_StatusTypeDef status;

    status = icm42688_writeReg(ICM42688_GYRO_CONFIG_0, reg_val);
    if (status != HAL_OK)
    {
        printf("[ICM42688] configGyro write failed\r\n");
        return HAL_ERROR;
    }

    icm42688_delay(1);

    status = icm42688_readReg(ICM42688_GYRO_CONFIG_0, &readback, 1);
    if (status != HAL_OK)
    {
        printf("[ICM42688] configGyro readback failed\r\n");
        return HAL_ERROR;
    }

    if (readback != reg_val)
    {
        printf("[ICM42688] configGyro mismatch: wrote 0x%02X read 0x%02X\r\n",
               reg_val, readback);
        return HAL_ERROR;
    }

    printf("[ICM42688] GYRO_CONFIG_0  = 0x%02X OK\r\n", readback);
    return HAL_OK;
}

/* =========================================================
 * icm42688_setup()
 * Full init sequence:
 *   1. Check connection (WHO_AM_I)
 *   2. Soft reset (write 0x01 to DEVICE_CONFIG, reg 0x11)
 *   3. Wake up (PWR_MGMT_0)
 *   4. Configure accel & gyro FS / ODR
 * ========================================================= */
HAL_StatusTypeDef icm42688_setup(icm42688_config_t cfg)
{
    printf("[ICM42688] ----- SETUP START -----\r\n");

    /* --- 1. WHO_AM_I --- */
    uint8_t id = icm42688_readWhoAmI();
    if (id != ICM42688_ID)
    {
        printf("[ICM42688] WHO_AM_I=0x%02X, expected 0x%02X — ABORT\r\n",
               id, ICM42688_ID);
        return HAL_ERROR;
    }
    printf("[ICM42688] WHO_AM_I=0x%02X OK\r\n", id);

    /* --- 2. Soft reset (DEVICE_CONFIG reg 0x11, bit 0) --- */
    icm42688_writeReg(0x11, 0x01);
    icm42688_delay(2);   /* datasheet: ≥1 ms after soft-reset */

    /* --- 3. Wake up --- */
    if (icm42688_wakeup() != HAL_OK)
    {
        return HAL_ERROR;
    }
    printf("[ICM42688] Wakeup OK\r\n");

    /* --- 4. Configure accel --- */
    if (icm42688_configAccel(cfg.accel_fs, cfg.accel_odr) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* --- 5. Configure gyro --- */
    if (icm42688_configGyro(cfg.gyro_fs, cfg.gyro_odr) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* After ODR / FS change, wait ≥1 ODR period (1 kHz → 1 ms) */
    icm42688_delay(10);

    printf("[ICM42688] ----- SETUP DONE -----\r\n");
    return HAL_OK;
}

/* =========================================================
 * icm42688_setGyroBias() / icm42688_getGyroBias()
 * Set or get the software gyro bias. These values are
 * subtracted inside icm42688_readRawSensor().
 * ========================================================= */
void icm42688_setGyroBias(int16_t bx, int16_t by, int16_t bz)
{
    s_gyro_bias_x = bx;
    s_gyro_bias_y = by;
    s_gyro_bias_z = bz;
}

void icm42688_getGyroBias(int16_t *bx, int16_t *by, int16_t *bz)
{
    *bx = s_gyro_bias_x;
    *by = s_gyro_bias_y;
    *bz = s_gyro_bias_z;
}

/* =========================================================
 * icm42688_readRawSensor()
 * Burst-read 12 bytes (accel X/Y/Z + gyro X/Y/Z).
 * Gyro bias (from calibration) is subtracted here.
 * ========================================================= */
HAL_StatusTypeDef icm42688_readRawSensor(icm42688_data *out)
{
    uint8_t buf[12];
    HAL_StatusTypeDef status;

    status = icm42688_readReg(ICM42688_ACCEL_DATA_X1, buf, 12);

    if (status != HAL_OK)
    {
        printf("[ICM42688] readRawSensor failed\r\n");
        return HAL_ERROR;
    }

    /* Big-endian: MSB first */
    out->x_accel = (int16_t)((buf[0]  << 8) | buf[1]);
    out->y_accel = (int16_t)((buf[2]  << 8) | buf[3]);
    out->z_accel = (int16_t)((buf[4]  << 8) | buf[5]);

    out->x_gyro  = (int16_t)((buf[6]  << 8) | buf[7])  - s_gyro_bias_x;
    out->y_gyro  = (int16_t)((buf[8]  << 8) | buf[9])  - s_gyro_bias_y;
    out->z_gyro  = (int16_t)((buf[10] << 8) | buf[11]) - s_gyro_bias_z;

    return HAL_OK;
}
