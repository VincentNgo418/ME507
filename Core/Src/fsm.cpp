/**
 * @file fsm.cpp
 * @brief Implementation of the Finite State Machine (FSM) controlling robot operation modes.
 *
 * This file defines the behavior of each operational mode of the Ping Pong Launcher Robot,
 * including initialization, manual control, target aiming, and automatic LIDAR-based targeting.
 *
 * @author Jason
 * @date June 5, 2025
 */

#include "fsm.h"
#include "main.h"
#include "motor_driver.h"
#include "servo_driver.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;

/**
 * @brief FSM constructor. Initializes the state machine to S0_INIT.
 */
FSM::FSM() : state(S0_INIT) { }

/**
 * @brief Sets the state of the FSM.
 * @param new_state The new state to transition to.
 */
void FSM::set_state(system_state_t new_state) {
    state = new_state;
}

/**
 * @brief Runs the current state's logic. Should be called repeatedly in the main loop.
 */
void FSM::run(void)
{
    switch (state)
    {
        case S0_INIT:
            state = S1_IDLE;
            break;

        case S1_IDLE:
            // Idle state - waiting for input
            break;

        case S2_MANUAL_STEP_INPUT:
            // Manual step-by-step control via UART commands (see main.c callback)
            break;

        case S3_MANUAL_TARGET:
            // Accepts absolute motor/servo commands via UART
            break;

        case S4_AUTOMATIC: {
            // Run this block every ~750 ms
            error = HAL_GetTick() - prevTick;
            if (error > UINT32_MAX / 2) {
                error -= UINT32_MAX;
            } else if (error < -static_cast<int32_t>(UINT32_MAX / 2)) {
                error += UINT32_MAX;
            }

            if (abs(error) > 750) {
                // Reset servo each cycle
                servo_duty(&servo_1, 2655);

                // --- Target Detection ---
                if (distance_cm < 200 && intensity_value > 2000) {
                    set_duty(&motor_1, 2500); // Start flywheel
                    HAL_UART_Transmit(&huart1, (uint8_t*)"Launching\r\n", 11, HAL_MAX_DELAY);
                    HAL_Delay(600);
                    servo_duty(&servo_1, 6000); // Fire
                    HAL_Delay(200);
                }

                // --- Patrol Movement (sweep turret back and forth) ---
                else {
                    switch (i) {
                        case 0:
                            motor_d_set_pos(&Pololu_2, &pos_controller_1, 400);
                            HAL_UART_Transmit(&huart1, (uint8_t*)"CW, 0\r\n", 7, HAL_MAX_DELAY);
                            i = 1;
                            break;
                        case 1:
                            motor_d_set_pos(&Pololu_2, &pos_controller_1, -400);
                            HAL_UART_Transmit(&huart1, (uint8_t*)"CCW, 1\r\n", 8, HAL_MAX_DELAY);
                            i = 2;
                            break;
                        case 2:
                            motor_d_set_pos(&Pololu_2, &pos_controller_1, -400);
                            HAL_UART_Transmit(&huart1, (uint8_t*)"CCW, 2\r\n", 8, HAL_MAX_DELAY);
                            i = 3;
                            break;
                        case 3:
                            motor_d_set_pos(&Pololu_2, &pos_controller_1, 400);
                            HAL_UART_Transmit(&huart1, (uint8_t*)"CW, 3\r\n", 7, HAL_MAX_DELAY);
                            i = 0;
                            break;
                    }
                }

                // --- Wind Down Flywheels when nothing detected---
                if (dynamic_duty > 200) {
                    dynamic_duty -= 300;
                    set_duty(&motor_1, dynamic_duty);
                    char buffer[40];
                    sprintf(buffer, "Current duty %ld \r\n", dynamic_duty);
                    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
                }

                prevTick = HAL_GetTick();
            }
            break;
        }

        default:
            HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid State\r\n", 16, HAL_MAX_DELAY);
            break;
    }
}

/**
 * @brief Set the manual movement direction for S2 manual mode.
 * @param dir -1 for left, 1 for right, 0 for stop. Is called in main.
 */
void FSM::set_move_dir(int8_t dir) {
    move_dir = dir;
}

/**
 * @brief Get the current movement direction.
 * @return Current movement direction. -1 for left, 1 for right, 0 for stop
 */
int8_t FSM::get_move_dir() {
    return move_dir;
}

/**
 * @brief Sets the robot's home heading for use in automatic mode.
 * @param current_heading The heading angle from the IMU to record as home.
 */
void FSM::set_home_heading(int16_t current_heading) {
    home_heading = current_heading;
    char buffer[40];
    sprintf(buffer, "Heading of %d received\r\n", current_heading);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
}
