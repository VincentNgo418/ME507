/*
 * motor_driver.c
 *
 *  Created on: Apr 24, 2025
 *      Author: Vincent
 */

#include "motor_driver.h"

//constructor
motor_t new_motor(TIM_HandleTypeDef* timer,uint32_t forward_channel, uint32_t backward_channel) {
    motor_t motor;
    motor.pulse = 0;
    motor.max_pulse = 4799;
    motor.timer = timer;
    motor.forward_channel = forward_channel;
    motor.backward_channel = backward_channel;
    HAL_TIM_PWM_Start(timer, forward_channel);
    HAL_TIM_PWM_Start(timer, backward_channel);
    return motor;
}

//brake
void brake(motor_t* motor) {
    __HAL_TIM_SET_COMPARE(motor->timer, motor->forward_channel, motor->max_pulse);
	__HAL_TIM_SET_COMPARE(motor->timer, motor->backward_channel, motor->max_pulse);
}


//coasting
void coast(motor_t* motor) {
    __HAL_TIM_SET_COMPARE(motor->timer, motor->forward_channel, 0);
	__HAL_TIM_SET_COMPARE(motor->timer, motor->backward_channel, 0);
}

//set motor speed
void set_duty(motor_t* motor, float percentage) {
    if(percentage < 0 && percentage >= -100) {
    	percentage = -percentage;
        __HAL_TIM_SET_COMPARE(motor->timer, motor->forward_channel, 0);
        __HAL_TIM_SET_COMPARE(motor->timer, motor->backward_channel, percentage*motor->max_pulse/100);

    }
    else if(percentage >=0 && percentage <= 100) {
        __HAL_TIM_SET_COMPARE(motor->timer, motor->forward_channel, percentage*motor->max_pulse/100);
        __HAL_TIM_SET_COMPARE(motor->timer, motor->backward_channel, 0);
    }
    else{
        //do nothing for now
    }
}
