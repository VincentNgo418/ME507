/*
 * motor_driver.c
 *
 *  Created on: Apr 28, 2025
 *      Author: Jason
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
    .PWM_CHANNEL_1 = TIM_CHANNEL_2,
    .htim = &htim1
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
	.prevTick = 0,
	.prevPulse = 0
};

// Controller 1
PI_Controller pos_controller_2 = {
    .Kp = 3.4f,
    .Ki = 0.0f,
    .integral = 0,
    .setpoint = 0,
	.prevTick = 0,
	.prevPulse = 0
};




void set_duty(motor_t* motor, uint32_t pulse_1) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->PWM_CHANNEL_1, pulse_1);
}

void set_duty_dual(motor_dual* motor_d, uint32_t pulse_1, uint32_t pulse_2) {
	__HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, pulse_1);
	__HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, pulse_2);
}

void motor_brake_dual(motor_dual* motor_d) {
	__HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, 4999);
	__HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, 4999);
}


void step_motor_forward(void) {
	set_duty_dual(&Pololu_2, 2500, 0);
	HAL_Delay(50);
	motor_brake_dual(&Pololu_2);
}

void step_motor_backward(void) {
	set_duty_dual(&Pololu_2, 0, 2500);
	HAL_Delay(50);
	motor_brake_dual(&Pololu_2);
}

void launch(void) {

	servo_duty(&servo_1,8275);
	HAL_Delay(300);
	servo_duty(&servo_1,1655);


}

//Sets
//@param motor_dual*, a motor with two PWM inputs
//@param PI_Controller*, controller
const int32_t ENCODER_MAX = 65536;
void motor_d_update_pos(motor_dual* motor_d, PI_Controller* ctrl) {
    int32_t current_count = __HAL_TIM_GET_COUNTER(motor_d->enc);
    int32_t error = ctrl->setpoint - current_count;



	if (error > ENCODER_MAX / 2) {
		error -= ENCODER_MAX; // Subtract resolution for positive wrap-around
	} else if (error < -ENCODER_MAX / 2) {
		error += ENCODER_MAX; // Add resolution for negative wrap-around
	}

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

    //saturate jump in pulse
	if (pulse - ctrl->prevPulse > 2000) {
		pulse = ctrl->prevPulse + 2000;
	}
	if (pulse - ctrl->prevPulse < -2000) {
		pulse = ctrl->prevPulse - 2000;
	}

	const int16_t PULSE_LIMIT = 3500;
    //saturate pulse
    if (pulse > PULSE_LIMIT) pulse = PULSE_LIMIT;
    if (pulse < -PULSE_LIMIT) pulse = -PULSE_LIMIT;


    ctrl->prevPulse = pulse;

    if (control > 4) {
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, pulse);
    } else if (control < -4) {
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, abs(pulse));
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, 0);
    } else {
        // Stop motor if within small error band
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, 4999);
        __HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, 4999);
    }

    if (error < 50 && error > -50) {
    	__HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_1, 4999);
    	__HAL_TIM_SET_COMPARE(motor_d->htim, motor_d->PWM_CHANNEL_2, 4999);
    }
}

void motor_d_set_pos(motor_dual* motor_d, PI_Controller* ctrl, int32_t pos) {
	ctrl->setpoint = pos + ctrl->setpoint;
	if (ctrl->setpoint > ENCODER_MAX) {
		ctrl->setpoint = ctrl->setpoint - ENCODER_MAX;
	}
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



