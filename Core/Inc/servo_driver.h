/*
 * servo_driver.h
 *
 *  Created on: May 15, 2025
 *      Author: Jason
 */

#ifndef INC_SERVO_DRIVER_H_
#define INC_SERVO_DRIVER_H_

#include "stm32f4xx_hal.h"

typedef struct servo {
	uint32_t PWM_CHANNEL;
	TIM_HandleTypeDef* htim;
} servo_t;

extern servo_t servo_1;


#ifdef __cplusplus
extern "C" {
#endif

void servo_duty(servo_t* servo, uint32_t pulse_1);
void servo_brake(servo_t* servo);
void servo_disable(servo_t* servo);


#ifdef __cplusplus
} // End of extern "C" block
#endif



#endif /* INC_SERVO_DRIVER_H_ */
