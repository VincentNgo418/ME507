/*
 * servo_driver.c
 *
 *  Created on: May 15, 2025
 *      Author: Jason
 */



#include "servo_driver.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

servo_t servo_1 = {
	.PWM_CHANNEL = TIM_CHANNEL_4,
	.htim = &htim1
};



void servo_duty(servo_t* servo, uint32_t pulse_1) {
    __HAL_TIM_SET_COMPARE(servo->htim, servo->PWM_CHANNEL, pulse_1);
}

void servo_brake(servo_t* servo) {
    __HAL_TIM_SET_COMPARE(servo->htim, servo->PWM_CHANNEL, 0);
}

void servo_disable(servo_t* servo) {
    __HAL_TIM_SET_COMPARE(servo->htim, servo->PWM_CHANNEL, 0);
}

