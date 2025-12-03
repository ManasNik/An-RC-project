#include "Pedals.h"

void throttle_pedal(uint16_t* throttle_input, uint16_t* throttle_output,uint8_t* accel_const, uint8_t* decel_const){
	if(*throttle_input > *throttle_output){
		// Increasing throttle to match the input.
		if(*throttle_input - *throttle_output != 0){
			*throttle_output += *accel_const;
		}
	}
	else{
		// Decreasing throttle to match the input.
		if(*throttle_output - *throttle_input != 0){
			*throttle_output -= *decel_const;
		}
	}
}

void brake_pedal(uint8_t* brake_input, uint8_t* brake_force){
	switch(*brake_input){
		case 1: *brake_force = 45;
		break;

		case 2: *brake_force = 47;
		break;

		case 3: *brake_force = 49;
		break;

		case 4: *brake_force = 51;
		break;

		default: *brake_force = 0;
		break;
	}
}
