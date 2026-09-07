/*
 * icm42688_stm32.h
 *
 *  Created on: Jul 1, 2026
 *      Author: User
 */

#ifndef INC_ICM42688_STM32_H_
#define INC_ICM42688_STM32_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "icm42688.h"
#include <stdio.h>


I2C_HandleTypeDef *_icm42688_i2c_port;

void icm42688_assignI2C(I2C_HandleTypeDef *hi2c_device)
{
    _icm42688_i2c_port = hi2c_device;
}


void icm42688_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

/* ---------------------------------------------------------------
 * icm42688_writeReg()
 * Write one byte to a register (Bank 0 address space).
 * --------------------------------------------------------------- */
HAL_StatusTypeDef icm42688_writeReg(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(
        _icm42688_i2c_port,
        ICM42688_I2C_ADDR << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );

    if (status != HAL_OK)
    {
        printf("[ICM42688] writeReg(0x%02X) failed, HAL=%d\r\n", reg, status);

        uint32_t err = HAL_I2C_GetError(_icm42688_i2c_port);
        if (err & HAL_I2C_ERROR_BERR)   printf("[ICM42688] I2C BUS ERROR\r\n");
        if (err & HAL_I2C_ERROR_ARLO)   printf("[ICM42688] I2C ARBITRATION LOST\r\n");
        if (err & HAL_I2C_ERROR_AF)     printf("[ICM42688] I2C ACK FAILURE\r\n");
        if (err & HAL_I2C_ERROR_OVR)    printf("[ICM42688] I2C OVERRUN\r\n");
        if (err & HAL_I2C_ERROR_TIMEOUT)printf("[ICM42688] I2C TIMEOUT\r\n");
    }

    return status;
}


HAL_StatusTypeDef icm42688_readReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(
        _icm42688_i2c_port,
        ICM42688_I2C_ADDR << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        100
    );

    if (status != HAL_OK)
    {
        printf("[ICM42688] readReg(0x%02X, len=%d) failed, HAL=%d\r\n",
               reg, len, status);
    }

    return status;
}

#ifdef __cplusplus
}
#endif

#endif /* INC_ICM42688_STM32_H_ */
