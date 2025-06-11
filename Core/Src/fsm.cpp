/*
 * fsm.cpp
 *
 *  Created on: Jun 5, 2025
 *      Author: Jason
 */


#include "fsm.h"
#include "main.h"
#include "motor_driver.h"
#include "servo_driver.h"
#include <string.h>   // for strlen, strcmp, etc.
#include <stdio.h>    // for sprintf
#include <stdlib.h>

extern UART_HandleTypeDef huart1;




FSM::FSM():state(S0_INIT){


}

void FSM::set_state(system_state_t new_state) {
    state = new_state;
}


void FSM::run(void)
{
    // Switch case is a bit tidier than a bunch of if / else if statements
    // but you must remember to use break or fall through into later states
    switch(state)
    {
        case S0_INIT:

        	//HAL_UART_Transmit(&huart1, (uint8_t*)"In State 0\r\n", 13, HAL_MAX_DELAY);
            state = S1_IDLE;
            break;

        case S1_IDLE:
        	//HAL_UART_Transmit(&huart1, (uint8_t*)"In State 1\r\n", 13, HAL_MAX_DELAY);

            break;

        case S2_MANUAL_STEP_INPUT:

            //HAL_UART_Transmit(&huart1, (uint8_t*)"In State 2\r\n", 13, HAL_MAX_DELAY);
        	/*if (move_dir == 1)

        		motor_d_set_pos(&Pololu_2, &pos_controller_1 , 500);
			else if (move_dir == -1)
				motor_d_set_pos(&Pololu_2 , &pos_controller_1 , -500);
			else
				motor_brake_dual(&Pololu_2);*/
            break;

        case S3_MANUAL_TARGET:
            //HAL_UART_Transmit(&huart1, (uint8_t*)"In State 3\r\n", 13, HAL_MAX_DELAY);

            break;

        //Cooperative patrolling which utilizes P-controller
        case S4_AUTOMATIC:
        	//only run code when deltaT >= 1 sec

        	//calculate deltaT
        	error = HAL_GetTick() - prevTick;
			if (error > 4,294,967,295  / 2) {
				error -= 4,294,967,295 ; // Subtract resolution for positive wrap-around
			} else if (error < -4,294,967,295  / 2) {
				error += 4,294,967,295 ; // Add resolution for negative wrap-around
			}
        	if (abs(error) > 750) {
        		//always reset servo position
        		servo_duty(&servo_1,2655);

        		//check for target
        		if (distance_cm < 200 && intensity_value > 2000) {
        			//if present begin launching blam blam!
					char msg2[] = "Launching\r\n";
					dynamic_duty = 2500;
					set_duty(&motor_1, dynamic_duty);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);
					HAL_Delay(600);
					servo_duty(&servo_1,6000);
					HAL_Delay(200);
				}

        		//else perform stepping motion
        		//essentially creating a loop
				//step 5 times to the right
				//step 5 times to the left
        		else {

            		if(i == 0) {
    					motor_d_set_pos(&Pololu_2, &pos_controller_1, 400);
    					char msg2[] = "CW, 0\r\n";
    					HAL_UART_Transmit(&huart1, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);
    					i = 1;
    				}
            		else if (i == 1) {
    					motor_d_set_pos(&Pololu_2, &pos_controller_1, -400);
    					char msg2[] = "CCW, 1\r\n";
						HAL_UART_Transmit(&huart1, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);
						i = 2;
    				}
            		else if (i == 2) {
						motor_d_set_pos(&Pololu_2, &pos_controller_1, -400);
						char msg2[] = "CCW, 2\r\n";
						HAL_UART_Transmit(&huart1, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);
						i = 3;
					}
            		if(i == 3) {
						motor_d_set_pos(&Pololu_2, &pos_controller_1, 400);
						char msg2[] = "CW, 3\r\n";
						HAL_UART_Transmit(&huart1, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);
						i = 0;
					}
            		//char stuff[30] = {0};
					//sprintf((char*)stuff, "Index i %d \r\n", i);
					//HAL_UART_Transmit(&huart1, (uint8_t*)stuff, strlen(stuff), 1000);
        		}

				//wind down behavior
				if ( dynamic_duty > 200) {
					dynamic_duty -= 300;
					set_duty(&motor_1, dynamic_duty);
					char stuff[30] = {0};
					sprintf((char*)stuff, "Current duty %d \r\n", dynamic_duty);
					HAL_UART_Transmit(&huart1, (uint8_t*)stuff, strlen(stuff), 1000);
				}



				prevTick = HAL_GetTick();
        	}


        	break;

            //HAL_UART_Transmit(&huart1, (uint8_t*)"In State 4\r\n", 13, HAL_MAX_DELAY);
        	/*if(home_heading == 0) {
        		char stuff[30] = {0};
        		sprintf((char*)stuff, "No heading found, val is %d\r\n", home_heading);
        		HAL_UART_Transmit(&huart1, (uint8_t*)stuff, strlen(stuff), 1000);
        		set_state(FSM::S1_IDLE);

        	}
        	else {
        		char stuff[30] = {0};
        		//sprintf((char*)stuff, "Four %d\r\n", s4flag);
				//HAL_UART_Transmit(&huart1, (uint8_t*)stuff, strlen(stuff), 1000);
        		int i = 0;*/
        		/*switch(s4flag){
        		        		//just entered auto mode,
        						//turn on motor
        						case 1:
        							//set_duty_dual(&Pololu_2, 3000, 0);
        							//maybe home the motor if feeling fancy
        							//set motor speed to certain amount
        							s4flag = 2;
        							break;
        						//spin (wait) until reaching the end of the travel (home-1440)
        						case 2:

									sprintf((char*)stuff, "Currently facing %d\r\n", heading);
									HAL_UART_Transmit(&huart1, (uint8_t*)stuff, strlen(stuff), 1000);
										//if position has met limit (180)
											//s4flag = 4;
										//check distance and intensity
										//if reqs met
											//s4flag = 3;
										//else



        							if (abs(heading - home_heading) > 500) {
        								sprintf((char*)stuff, "Passed limit! %d\r\n", heading);
        								HAL_UART_Transmit(&huart1, (uint8_t*)stuff, strlen(stuff), 1000);

        							}
        							break;
        						case 3:
        							//shoot the ball
        							//this can be blocking
        							//return to flag1
        							break;
        						case 4:
        							//make a "job complete!" statement
        							break;
        						default:
        							set_state(FSM::S1_IDLE);
        							break;
        					*/
        default:
                	HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid State\r\n", 16, HAL_MAX_DELAY);
                	break;

    }



    }







void FSM::set_move_dir(int8_t dir) {
	move_dir = dir;
}

int8_t FSM::get_move_dir() {
	return move_dir;
}

void FSM::set_home_heading(int16_t current_heading) {
	home_heading = current_heading;
	char stuff[30] = {0};
	sprintf((char*)stuff, "Heading of %d received\r\n", current_heading);
	HAL_UART_Transmit(&huart1, (uint8_t*)stuff, strlen(stuff), 1000);
}
