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
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim2;

#define SATURATION_LIMIT 4000

#define fabs(control_effort) \
    ((int16_t)((control_effort) > SATURATION_LIMIT ? SATURATION_LIMIT : \
              ((control_effort) < -SATURATION_LIMIT ? -SATURATION_LIMIT : (control_effort))))

// Flywheel Motor 1

motor_t motor_1 = {
    .PWM_CHANNEL_1 = TIM_CHANNEL_3,
    .htim = &htim3
};

// Flywheel Motor 2

motor_t motor_2 = {
    .PWM_CHANNEL_1 = TIM_CHANNEL_4,
    .htim = &htim3
};

// Pololu Motor 1

motor_dual Pololu_1 = {
	.PWM_CHANNEL_1 = TIM_CHANNEL_3,
	.PWM_CHANNEL_2 = TIM_CHANNEL_4,
	.htim = &htim5,
	.enc = &htim2

};

// Pololu Motor 2

motor_dual Pololu_2 = {
	.PWM_CHANNEL_1 = TIM_CHANNEL_1,
	.PWM_CHANNEL_2 = TIM_CHANNEL_2,
	.htim = &htim3,
	.enc = &htim4
};

// Controller 1

PI_Controller pos_controller_1 = {
    .Kp = 3.25f,
    .Ki = 0.0f,
    .integral = 0,
    .setpoint = 0,
	.prevTick = 0
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
//Sets
//@param motor_dual*, a motor with two PWM inputs
//@param PI_Controller*, controller

void motor_d_update_pos(motor_dual* motor_d, PI_Controller* ctrl) {
    int32_t current_count = __HAL_TIM_GET_COUNTER(motor_d->enc);
    int32_t error = ctrl->setpoint - current_count;

    //calculate deltaT
   	uint32_t deltaT = HAL_GetTick() - ctrl->prevTick;
   	ctrl->prevTick = HAL_GetTick();

    // Update integral
    ctrl->integral += error*deltaT*0.001;

    // Anti-windup (optional)
    if (ctrl->integral > 10000) ctrl->integral = 10000;
    if (ctrl->integral < -10000) ctrl->integral = -10000;

    // PI output
    float control = ctrl->Kp * error + ctrl->Ki * ctrl->integral;

    // Convert to PWM pulse
    int16_t pulse = fabs(control);
    if (pulse > 4799) pulse = 4799;

    // Set direction based on sign
    if (control > 4) {
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, pulse);
    } else if (control < -4) {
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, 0);
    } else {
        // Stop motor if within small error band
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, 0);
    }
}


void motor_d_set_pos(motor_dual* motor_d, PI_Controller* ctrl, uint32_t pos) {
	ctrl->setpoint = pos + motor_d_get_pos(motor_d);

}

uint32_t motor_d_get_pos(motor_dual* motor_d) {
	return __HAL_TIM_GET_COUNTER(motor_d->enc);
}

void motor_brake(motor_t* motor) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, 0);
}

void motor_disable(motor_t* motor) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, 0);
}



