/**
 * @file motor_driver.h
 * @brief Header file for motor driver control on STM32 using PWM and encoder feedback.
 *
 * This module provides an abstraction layer for controlling single-channel and dual-channel motors,
 * implementing both open-loop PWM and closed-loop PID-based positioning with encoders.
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "stm32f4xx_hal.h"

/**
 * @brief Struct for single-channel PWM motor.
 */
typedef struct motor {
    uint32_t PWM_CHANNEL_1;           /**< PWM channel used for the motor */
    TIM_HandleTypeDef* htim;          /**< Timer handle for PWM generation */
} motor_t;

/**
 * @brief Struct for dual-channel motor with encoder.
 */
typedef struct motor_d {
    uint32_t PWM_CHANNEL_1;           /**< First PWM output channel */
    uint32_t PWM_CHANNEL_2;           /**< Second PWM output channel */
    TIM_HandleTypeDef* htim;          /**< Timer handle for PWM generation */
    TIM_HandleTypeDef* enc;           /**< Timer handle for encoder input */
} motor_dual;

/**
 * @brief Proportional-Integral controller used for position control.
 */
typedef struct controller {
    float Kp;                         /**< Proportional gain */
    float Ki;                         /**< Integral gain */
    float integral;                   /**< Accumulated integral value */
    int32_t setpoint;                 /**< Desired motor position */
    uint32_t prevTick;                /**< Timestamp of last control update */
    int16_t prevPulse;                /**< Previous encoder pulse count */
} PI_Controller;

// === Global Motor and Controller Instances ===

/**
 * @brief Global motor instance (single-channel)
 */
extern motor_t motor_1;

/**
 * @brief Global motor instance (single-channel)
 */
extern motor_t motor_2;

/**
 * @brief Dual-channel motor with encoder (typically Pololu motor)
 */
extern motor_dual Pololu_1;

/**
 * @brief Dual-channel motor with encoder (typically Pololu motor)
 */
extern motor_dual Pololu_2;

/**
 * @brief Position controller instance for closed-loop control
 */
extern PI_Controller pos_controller_1;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set PWM duty cycle on a single motor.
 * @param motor Pointer to motor structure.
 * @param pulse_1 Pulse width (0 to timer ARR).
 */
void set_duty(motor_t* motor, uint32_t pulse_1);

/**
 * @brief Brake a single-channel motor by grounding both terminals.
 * @param motor Pointer to motor structure.
 */
void motor_brake(motor_t* motor);

/**
 * @brief Disable a motor by setting outputs low or floating.
 * @param motor Pointer to motor structure.
 */
void motor_disable(motor_t* motor);

/**
 * @brief Set PWM duty cycle on a dual-channel motor.
 * @param motor_d Pointer to dual-motor structure.
 * @param pulse_1 PWM for channel 1.
 * @param pulse_2 PWM for channel 2.
 */
void set_duty_dual(motor_dual* motor_d, uint32_t pulse_1, uint32_t pulse_2);

/**
 * @brief Brake a dual-channel motor.
 * @param motor_d Pointer to motor structure.
 */
void motor_brake_dual(motor_dual* motor_d);

/**
 * @brief Update position of motor using PI control.
 * @param motor_d Pointer to motor.
 * @param ctrl Pointer to PI controller.
 */
void motor_d_update_pos(motor_dual* motor_d, PI_Controller* ctrl);

/**
 * @brief Command motor to move to a specific position.
 * @param motor_d Pointer to motor.
 * @param ctrl Pointer to controller.
 * @param pos Desired setpoint position.
 */
void motor_d_set_pos(motor_dual* motor_d, PI_Controller* ctrl, int32_t pos);

/**
 * @brief Get current encoder position.
 * @param motor_d Pointer to motor.
 * @return Current position as a 32-bit signed value.
 */
uint32_t motor_d_get_pos(motor_dual* motor_d);

/**
 * @brief Rotate motor a single step backward (left).
 */
void step_motor_backward(void);

/**
 * @brief Rotate motor a single step forward (right).
 */
void step_motor_forward(void);

/**
 * @brief Trigger the launch mechanism (flywheel or servo).
 */
void launch(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_H
