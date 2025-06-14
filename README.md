# ME 507 - Ping Pong Ball Launcher

**Team Members:** Vincent Ngo, Jason Wong  
**Support:** Charlie Refvem

## Overview

This repository contains code used for the Ping Pong Ball Launcher for ME507.

**Documentation:**  
[View the Doxygen Documentation](https://vincentngo418.github.io/ME507/)

**Core Files:** 
- `main.c`/`main.h`: Contains the main control loop, which calls the FSM fsm.run().
- `motor_driver.c`/`motor_driver.h`: Contains driver functions for single-channel and dual-channel DC motors.
- `servo_driver.c`/`servo_driver.h`: Provides control for the servo motor that adjusts launch angle.
- `fsm.cpp`/`fsm.h`: Implements the Finite State Machine (FSM) used to manage control modes.
- `bno055.c`/`bno055.h` Driver code for the BNO055 IMU sensor connected via I2C.
