/**
 * @file motor_driver.c
 * @brief Implements motor and controller functions for flywheels and positioning motors.
 *
 * This file contains the logic to control single-channel and dual-channel motors using PWM.
 * It supports basic operations (set duty, brake, disable) and PI control for accurate position feedback using encoders.
 *
 * @author Jason
 * @date April 28, 2025
 */

#include "motor_driver.h"
#include "main.h"
#include "servo_driver.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;

#define SATURATION_LIMIT 4000

/// Clamp macro with symmetric saturation
#define fabs(control_effort) \
    ((int16_t)((control_effort) > SATURATION_LIMIT ? SATURATION_LIMIT : \
              ((control_effort) < -SATURATION_LIMIT ? -SATURATION_LIMIT : (control_effort))))

/** @brief Flywheel motor (Left) */
motor_t motor_1 = {
    .PWM_CHANNEL_1 = TIM_CHANNEL_3,
    .htim = &htim3
};

/** @brief Flywheel motor (Right) */
motor_t motor_2 = {
    .PWM_CHANNEL_1 = TIM_CHANNEL_2,
    .htim = &htim1
};

/** @brief Turret pitch positioning motor */
motor_dual Pololu_1 = {
    .PWM_CHANNEL_1 = TIM_CHANNEL_3,
    .PWM_CHANNEL_2 = TIM_CHANNEL_4,
    .htim = &htim5,
    .enc = &htim2
};

/** @brief Turret heading positioning motor */
motor_dual Pololu_2 = {
    .PWM_CHANNEL_1 = TIM_CHANNEL_1,
    .PWM_CHANNEL_2 = TIM_CHANNEL_2,
    .htim = &htim3,
    .enc = &htim4
};

/** @brief PI controller for Pololu motor 1 */
PI_Controller pos_controller_1 = {
    .Kp = 3.25f,
    .Ki = 0.0f,
    .integral = 0,
    .setpoint = 0,
    .prevTick = 0,
    .prevPulse = 0
};

/** @brief PI controller for Pololu motor 2 */
PI_Controller pos_controller_2 = {
    .Kp = 3.4f,
    .Ki = 0.0f,
    .integral = 0,
    .setpoint = 0,
    .prevTick = 0,
    .prevPulse = 0
};

/**
 * @brief Set PWM duty for a single-channel motor.
 * @param motor Pointer to the motor_t structure.
 * @param pulse_1 PWM compare value, currently limited at 4999.
 */
void set_duty(motor_t* motor, uint32_t pulse_1) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, pulse_1);
}

/**
 * @brief Set PWM duty for a dual-channel motor.
 * @param motor_d Pointer to dual motor structure.
 * @param pulse_1 PWM for channel 1.
 * @param pulse_2 PWM for channel 2.
 */
void set_duty_dual(motor_dual* motor_d, uint32_t pulse_1, uint32_t pulse_2) {
    __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, pulse_1);
    __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, pulse_2);
}

/**
 * @brief Brakes a dual-channel motor by setting both channels high.
 * @param motor_d Pointer to dual motor.
 */
void motor_brake_dual(motor_dual* motor_d) {
    __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, 4999);
    __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, 4999);
}

/**
 * @brief Moves Pololu_2 a short distance forward.
 */
void step_motor_forward(void) {
    set_duty_dual(&Pololu_2, 2500, 0);
    HAL_Delay(50);
    motor_brake_dual(&Pololu_2);
}

/**
 * @brief Moves Pololu_2 a short distance backward.
 */
void step_motor_backward(void) {
    set_duty_dual(&Pololu_2, 0, 2500);
    HAL_Delay(50);
    motor_brake_dual(&Pololu_2);
}

/**
 * @brief Performs a launch sequence using a servo motor.
 */
void launch(void) {
    servo_duty(&servo_1, 8275);  // Launch
    HAL_Delay(300);
    servo_duty(&servo_1, 1655);  // Reset
}

/**
 * @brief Updates position of dual motor using PI control.
 * @param motor_d Pointer to dual motor.
 * @param ctrl Pointer to PI controller.
 */
void motor_d_update_pos(motor_dual* motor_d, PI_Controller* ctrl) {
    const int32_t ENCODER_MAX = 65536;

    int32_t current_count = __HAL_TIM_GET_COUNTER(motor_d->enc);
    int32_t error = ctrl->setpoint - current_count;

    if (error > ENCODER_MAX / 2) error -= ENCODER_MAX;
    else if (error < -ENCODER_MAX / 2) error += ENCODER_MAX;

    uint32_t deltaT = HAL_GetTick() - ctrl->prevTick;
    ctrl->prevTick = HAL_GetTick();

    ctrl->integral += error * deltaT * 0.001;

    if (ctrl->integral > 10000) ctrl->integral = 10000;
    if (ctrl->integral < -10000) ctrl->integral = -10000;

    float control = ctrl->Kp * error + ctrl->Ki * ctrl->integral;
    int16_t pulse = fabs(control);

    if (pulse - ctrl->prevPulse > 2000) pulse = ctrl->prevPulse + 2000;
    if (pulse - ctrl->prevPulse < -2000) pulse = ctrl->prevPulse - 2000;

    const int16_t PULSE_LIMIT = 3500;
    if (pulse > PULSE_LIMIT) pulse = PULSE_LIMIT;

    ctrl->prevPulse = pulse;

    if (control > 4) {
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, pulse);
    } else if (control < -4) {
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, 0);
    } else {
        motor_brake_dual(motor_d);
    }

    if (error < 50 && error > -50) {
        motor_brake_dual(motor_d);
    }
}

/**
 * @brief Sets position control target for dual motor relative to current position.
 * @param motor_d Pointer to motor.
 * @param ctrl Pointer to controller.
 * @param pos Relative step to add to setpoint.
 */
void motor_d_set_pos(motor_dual* motor_d, PI_Controller* ctrl, int32_t pos) {
    ctrl->setpoint += pos;
    if (ctrl->setpoint > 65536) {
        ctrl->setpoint -= 65536;
    }
}

/**
 * @brief Reads current encoder position of dual motor.
 * @param motor_d Pointer to motor.
 * @return Current encoder count.
 */
uint32_t motor_d_get_pos(motor_dual* motor_d) {
    return __HAL_TIM_GET_COUNTER(motor_d->enc);
}

/**
 * @brief Brakes a single-channel motor.
 * @param motor Pointer to motor.
 */
void motor_brake(motor_t* motor) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, 0);
}

/**
 * @brief Disables a single-channel motor.
 * @param motor Pointer to motor.
 */
void motor_disable(motor_t* motor) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, 0);
}
