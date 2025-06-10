/*
 * bno055_hal.c
 *
 *  Created on: Jun 4, 2025
 *      Author: Vincent
 */


#include "stm32f4xx_hal.h"
#include "bno055.h"

extern I2C_HandleTypeDef hi2c3;

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 wr_len)
{
    if (HAL_I2C_Mem_Write(&hi2c3, (dev_addr << 1), reg_addr, 0b1, reg_data, wr_len, HAL_MAX_DELAY) == HAL_OK)
        return 0;
    else
        return 1;
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 r_len)
{
    if (HAL_I2C_Mem_Read(&hi2c3, (dev_addr << 1), reg_addr, 0b1, reg_data, r_len, HAL_MAX_DELAY) == HAL_OK){
        return 0;
    }
    return 1;
}

void BNO055_delay_msec(u32 msec)
{
    HAL_Delay(msec);
}


void BNO055_setup(struct bno055_t* bno055_device)
{
    bno055_device ->dev_addr = BNO055_I2C_ADDR1;
    bno055_device->bus_write = BNO055_I2C_bus_write;
    bno055_device->bus_read = BNO055_I2C_bus_read;
    bno055_device->delay_msec = BNO055_delay_msec;

    if (bno055_init(bno055_device) == 0)
    {
        // Initialization successful
    }
}
