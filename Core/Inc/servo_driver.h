/**
 * @file servo_driver.h
 * @brief Header file for controlling servo motors using PWM on STM32.
 *
 * This module defines a servo structure and provides functions to set duty cycle,
 * brake, or disable the servo. It is tailored for use with STM32 timers and channels.
 */

#ifndef INC_SERVO_DRIVER_H_
#define INC_SERVO_DRIVER_H_

#include "stm32f4xx_hal.h"

/**
 * @brief Struct for servo configuration.
 *
 * Represents a single servo motor controlled via a specified PWM channel and timer.
 */
typedef struct servo {
	uint32_t PWM_CHANNEL;            /**< PWM channel assigned to the servo */
	TIM_HandleTypeDef* htim;         /**< Timer handle used for PWM generation */
} servo_t;

/**
 * @brief Global instance of servo motor (e.g., used for angle adjustment or launching).
 */
extern servo_t servo_1;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set the PWM duty cycle for the servo.
 * @param servo Pointer to the servo structure.
 * @param pulse_1 PWM pulse width (in timer ticks) to control the servo angle.
 */
void servo_duty(servo_t* servo, uint32_t pulse_1);

/**
 * @brief Apply brake or stop signal to the servo.
 * @param servo Pointer to the servo structure.
 */
void servo_brake(servo_t* servo);

/**
 * @brief Disable the servo output (sets PWM to 0 or idle).
 * @param servo Pointer to the servo structure.
 */
void servo_disable(servo_t* servo);

#ifdef __cplusplus
} // End of extern "C" block
#endif

#endif /* INC_SERVO_DRIVER_H_ */
