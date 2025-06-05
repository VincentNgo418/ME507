/*
 * motor_driver.h
 *
 *  Created on: Apr 24, 2025
 *      Author: Vincent, Jason
 */
#include <stdint.h>
#include <stm32f4xx_hal.h>
#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_


// Define the struct template
typedef struct {
    TIM_HandleTypeDef*  timer;
    uint32_t   forward_channel;
    uint32_t  backward_channel;
    uint16_t  pulse;
    uint16_t  max_pulse;
} motor_t;

//constructor
motor_t new_motor(TIM_HandleTypeDef* timer,uint32_t forward_channel, uint32_t backward_channel);

//brake
void brake(motor_t* motor);

//coasting
void coast(motor_t* motor);

//set motor speed
void set_duty(motor_t* motor, float percentage);

#endif /* INC_MOTOR_DRIVER_H_ */
