/**
 * @file servo_driver.c
 * @brief Control functions for servo motors using STM32 HAL.
 *
 * Provides abstraction for setting, braking, and disabling a PWM-driven servo motor.
 * Intended for use in applications like robotic actuation or launching mechanisms.
 *
 * @author Jason
 * @date May 15, 2025
 */

#include "servo_driver.h"
#include "main.h"

/// Global instance of servo motor 1 connected to TIM1, channel 4.
servo_t servo_1 = {
    .PWM_CHANNEL = TIM_CHANNEL_4,
    .htim = &htim1
};

/**
 * @brief Set the PWM duty cycle (pulse width) for a servo motor.
 * 
 * This function controls the position of the servo by setting its PWM pulse width.
 * Typical values range from ~1000 (0°) to ~9000 (180°), depending on the timer settings and servo model.
 *
 * @param servo Pointer to the servo instance.
 * @param pulse_1 Pulse width value to set on the PWM channel.
 */
void servo_duty(servo_t* servo, uint32_t pulse_1) {
    __HAL_TIM_SET_COMPARE(servo->htim, servo->PWM_CHANNEL, pulse_1);
}

/**
 * @brief Apply braking (0% duty) to stop servo signal.
 *
 * This will stop the signal to the servo, typically freezing its last position.
 *
 * @param servo Pointer to the servo instance.
 */
void servo_brake(servo_t* servo) {
    __HAL_TIM_SET_COMPARE(servo->htim, servo->PWM_CHANNEL, 0);
}

/**
 * @brief Fully disable the servo signal (same as brake).
 *
 * Intended to be used when turning off the servo in a safe shutdown or idle state.
 *
 * @param servo Pointer to the servo instance.
 */
void servo_disable(servo_t* servo) {
    __HAL_TIM_SET_COMPARE(servo->htim, servo->PWM_CHANNEL, 0);
}
