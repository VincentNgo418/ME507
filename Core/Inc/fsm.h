/*
 * fsm.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Jason
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_

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

	  system_state_t get_state() const { return this->state; }


private:
	  system_state_t state;

};





#endif /* INC_FSM_H_ */
