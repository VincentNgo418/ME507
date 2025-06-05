/*
 * motor_driver.c
 *
 *  Created on: Apr 28, 2025
 *      Author: Jason
 */



#include "motor_driver.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;


/* Flywheel Motor 1 */

motor_t motor_1 = {
    .PWM_CHANNEL_1 = TIM_CHANNEL_3,
    .htim = &htim3
};

/* Flywheel Motor 2 */

motor_t motor_2 = {
    .PWM_CHANNEL_1 = TIM_CHANNEL_4,
    .htim = &htim3
};

/* Pololu Motor 1 */

motor_dual Pololu_1 = {
	.PWM_CHANNEL_1 = TIM_CHANNEL_3,
	.PWM_CHANNEL_2 = TIM_CHANNEL_4,
	.htim = &htim5
};

/* Pololu Motor 2 */

motor_dual Pololu_2 = {
	.PWM_CHANNEL_1 = TIM_CHANNEL_1,
	.PWM_CHANNEL_2 = TIM_CHANNEL_2,
	.htim = &htim3
};




void set_duty(motor_t* motor, uint32_t pulse_1) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, pulse_1);
}

void set_duty_dual(motor_dual* motor_d, uint32_t pulse_1, uint32_t pulse_2) {
	__HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, pulse_1);
	__HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, pulse_2);
}

void motor_brake_dual(motor_dual* motor_d) {
	__HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, 4799);
	__HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, 4799);
}

void motor_brake(motor_t* motor) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, 0);
}

void motor_disable(motor_t* motor) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, 0);
}



