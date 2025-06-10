/*
 * bno055_hal.c
 *
 *  Created on: Jun 4, 2025
 *      Author: Vincent
 */

#ifndef __BNO055_HAL_H__
#define __BNO055_HAL_H__
#include "stm32f4xx_hal.h"
#include "../Drivers/BNO055/bno055.h"

#ifdef __cplusplus
extern "C" {
#endif

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 wr_len);
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 r_len);
void BNO055_setup(struct bno055_t* bno055_device);
void BNO055_delay_msec(u32 msec);

#ifdef __cplusplus
} // End of extern "C" block
#endif

#endif //__BNO055_HAL_H__
