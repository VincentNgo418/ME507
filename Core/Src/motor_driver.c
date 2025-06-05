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

motor_t motor_1 = {
    .PWM_CHANNEL_1 = TIM_CHANNEL_3,
    .PWM_CHANNEL_2 = TIM_CHANNEL_4,
    .htim = &htim5
};

motor_t motor_2 = {
    .PWM_CHANNEL_1 = TIM_CHANNEL_1,
    .PWM_CHANNEL_2 = TIM_CHANNEL_2,
    .htim = &htim3
};



void set_duty(motor_t* motor, uint32_t pulse_1, uint32_t pulse_2) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, pulse_1);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_2, pulse_2);
}



void motor_brake(motor_t* motor) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, 4799);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_2, 4799);
}

void motor_disable(motor_t* motor) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_2, 0);
}



