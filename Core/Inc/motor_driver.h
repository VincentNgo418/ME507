#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "stm32f4xx_hal.h"


typedef struct motor {
    uint32_t       PWM_CHANNEL_1;
    TIM_HandleTypeDef* htim;
} motor_t;

typedef struct motor_d {
    uint32_t       PWM_CHANNEL_1;
    uint32_t  	   PWM_CHANNEL_2;
    TIM_HandleTypeDef* htim;
} motor_dual;


extern motor_t motor_1;
extern motor_t motor_2;
extern motor_dual Pololu_1;
extern motor_dual Pololu_2;


void set_duty(motor_t* motor, uint32_t pulse_1);
void motor_brake(motor_t* motor);
void motor_disable(motor_t* motor);
void set_duty_dual(motor_dual* motor_d, uint32_t pulse_1, uint32_t pulse_2);
void motor_brake_dual(motor_dual* motor_d);


#endif // MOTOR_DRIVER_H



