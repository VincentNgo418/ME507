/*
 * fsm.cpp
 *
 *  Created on: Jun 5, 2025
 *      Author: Jason
 */


#include "fsm.h"
#include "main.h"
extern UART_HandleTypeDef huart1;




FSM::FSM():state(S0_INIT){


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

            break;

        case S3_MANUAL_TARGET:
            //HAL_UART_Transmit(&huart1, (uint8_t*)"In State 3\r\n", 13, HAL_MAX_DELAY);

            break;

        case S4_AUTOMATIC:
            //HAL_UART_Transmit(&huart1, (uint8_t*)"In State 4\r\n", 13, HAL_MAX_DELAY);

            break;




        default:
        	HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid State\r\n", 16, HAL_MAX_DELAY);
        	break;
    }


}


void FSM::set_state(system_state_t new_state) {
    state = new_state;
}



