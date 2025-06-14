/**
 * @file fsm.h
 * @brief Header file for the Finite State Machine (FSM) controlling robot operation modes.
 *
 * Defines the FSM class which manages the state transitions and behaviors for manual and
 * automatic control modes of the robot.
 *
 * @author Jason
 * @date June 5, 2025
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_

#include <stdint.h>

/**
 * @class FSM
 * @brief Finite State Machine for managing the robot's control logic.
 *
 * The FSM handles different control states such as initialization, manual control,
 * target-based motion, and automatic tracking using sensor feedback.
 */
class FSM {
public:
    /**
     * @enum system_state_t
     * @brief Enum representing FSM control states.
     */
    typedef enum state {
        S0_INIT,                /**< System initialization */
        S1_IDLE,                /**< Idle mode, awaiting input */
        S2_MANUAL_STEP_INPUT,   /**< Manual movement via key inputs */
        S3_MANUAL_TARGET,       /**< Manual motor/servo target input */
        S4_AUTOMATIC            /**< Autonomous operation using sensors */
    } system_state_t;

    /**
     * @brief Constructor for FSM class.
     */
    FSM();

    /**
     * @brief Executes logic based on current FSM state.
     */
    void run(void);

    /**
     * @brief Sets the FSM to a new state.
     * @param new_state The state to transition to.
     */
    void set_state(system_state_t new_state);

    /**
     * @brief Sets the motor movement direction (used in manual mode).
     * @param dir Direction of movement: -1 = left, 0 = stop, 1 = right.
     */
    void set_move_dir(int8_t dir);

    /**
     * @brief Gets the current movement direction.
     * @return Direction: -1, 0, or 1.
     */
    int8_t get_move_dir();

    /**
     * @brief Gets the current FSM state.
     * @return Current state.
     */
    system_state_t get_state() const;

    /**
     * @brief Sets the current IMU heading as the home reference.
     * @param home_heading Heading value to store as home.
     */
    void set_home_heading(int16_t home_heading);

private:
    system_state_t state;      /**< Current FSM state */
    int8_t move_dir;           /**< Motor movement direction */
    int16_t home_heading = 0;  /**< Reference IMU heading */
    int s4flag = 1;            /**< Internal state flag for S4 logic */
    uint32_t dynamic_duty = 0; /**< Variable duty for flywheels or future use */
    int i = 0;                 /**< Generic counter variable */
    uint32_t prevTick = 0;     /**< Stores previous timestamp (non-blocking delay logic) */
    int32_t error = 0;         /**< Placeholder for sensor or motor error values */
};

#endif /* INC_FSM_H_ */
