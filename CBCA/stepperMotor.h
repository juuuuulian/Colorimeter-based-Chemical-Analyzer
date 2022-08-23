/*
 * stepperMotor.h
 *
 *  Created on: Apr 9, 2022
 *      Author: julian
 */

#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_


void get_to_tube(uint8_t);
void home();
void init_motor();
void step_CW();
void step_CCW();
void set_position(uint8_t y);
void apply_phase(uint8_t x);


#endif /* STEPPERMOTOR_H_ */
