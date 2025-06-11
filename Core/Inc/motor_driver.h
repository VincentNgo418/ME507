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
    TIM_HandleTypeDef* enc;
} motor_dual;

typedef struct controller{
    float Kp;
    float Ki;
    float integral;
    int32_t setpoint;
    uint32_t prevTick;
    int16_t prevPulse;
} PI_Controller;




extern motor_t motor_1;
extern motor_t motor_2;
extern motor_dual Pololu_1;
extern motor_dual Pololu_2;
extern PI_Controller pos_controller_1;

void set_duty(motor_t* motor, uint32_t pulse_1);
void motor_brake(motor_t* motor);
void motor_disable(motor_t* motor);
void set_duty_dual(motor_dual* motor_d, uint32_t pulse_1, uint32_t pulse_2);
void motor_brake_dual(motor_dual* motor_d);

void motor_d_update_pos(motor_dual* motor_d, PI_Controller* ctrl);
void motor_d_set_pos(motor_dual* motor_d, PI_Controller* ctrl, uint32_t pos);
uint32_t motor_d_get_pos(motor_dual* motor_d);


#endif // MOTOR_DRIVER_H



