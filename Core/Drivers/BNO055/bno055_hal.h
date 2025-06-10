/*
 * bno055_hal.c
 *
 *  Created on: Jun 4, 2025
 *      Author: Vincent
 */

#ifndef __BNO055_HAL_H__
#define __BNO055_HAL_H__
#include "stm32f4xx_hal.h"
#include "bno055.h"


s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 wr_len);
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 r_len);
void BNO055_setup(struct bno055_t* bno055_device);
void BNO055_delay_msec(u32 msec);

#endif
