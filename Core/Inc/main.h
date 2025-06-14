/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   Contains shared defines, structures, and global variables.
 *
 * @author         : Jason
 * @date           : 2025
 * @version        : 1.0
 *
 * This file declares hardware handles, types, and global variables used across
 * the application, including user-defined structures and sensor data.
 *
 * @copyright
 * Copyright (c) 2025 STMicroelectronics.
 * This software is licensed under terms in the LICENSE file in the root directory.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 ******************************************************************************
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"  /**< HAL library for STM32F4 peripherals */

/**
 * @brief Custom data structure used for application-specific motor or timer management.
 */
typedef struct my_struct {
    int32_t field_1;                   /**< Custom 32-bit signed field (usage TBD) */
    uint16_t field_2;                  /**< Custom 16-bit unsigned field (usage TBD) */
    TIM_HandleTypeDef* htim;          /**< Pointer to a timer handle (used for PWM, encoder, etc.) */
} my_struct_t;

/**
 * @brief Distance reading from the TF-Luna LIDAR sensor (in centimeters).
 */
extern volatile uint16_t distance_cm;

/**
 * @brief Heading angle from the BNO055 IMU sensor (in degrees).
 */
extern volatile int16_t heading;

/**
 * @brief Intensity or reflectivity value from the TF-Luna LIDAR.
 */
extern volatile uint16_t intensity_value;


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/**
 * @brief General error handler. Called when a critical error occurs.
 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
