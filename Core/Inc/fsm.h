/*
 * fsm.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Jason
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_
#include <stdint.h>


class FSM {

public:

	typedef enum state {
		  S0_INIT,
		  S1_IDLE,
		  S2_MANUAL_STEP_INPUT,
		  S3_MANUAL_TARGET,
		  S4_AUTOMATIC
	  } system_state_t;

	  FSM();

	  void run(void);

	  void set_state(system_state_t new_state);
	  void set_move_dir(int8_t dir);
	  int8_t get_move_dir();
	  system_state_t get_state() const { return this->state; }
	  void set_home_heading(int16_t home_heading);

private:
	  system_state_t state;
	  int8_t move_dir;
	  int16_t home_heading = 0;
	  int s4flag = 1;
	  uint32_t dynamic_duty = 0;
	  int i = 0;
	  uint32_t prevTick = 0;
	  int32_t error = 0;
};





#endif /* INC_FSM_H_ */
