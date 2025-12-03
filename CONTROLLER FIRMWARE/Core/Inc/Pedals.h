/*
 * Pedals.h
 *
 * Created on: Sep 4, 2025
 * Author: manas
 *
 * This file contains all function declarations of the control logic related to throttle and braking.
 */

#ifndef INC_PEDALS_H_
#define INC_PEDALS_H_

#include "stdint.h"

void throttle_pedal(uint16_t* throttle_input, uint16_t* throttle_output,uint8_t* accel_const, uint8_t* decel_const);
void brake_pedal(uint16_t* brake_input, uint8_t* brake_output);

#endif /* INC_PEDALS_H_ */
