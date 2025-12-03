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

void brake_pedal(uint16_t* brake_input, uint8_t* brake_output){
	// Not actually needed but to prevent noise from causing unwanted braking.
	if(*brake_input < 10){
		*brake_input = 0;
	}

	/* Brake output decides the value by which the throttle has to reduce to slow down the motors.
	 * 1 is slow braking and 5 is fast braking.
	*/
	if(*brake_input > 10 && *brake_input <= 100){
		*brake_output = 1;
	}
	else if(*brake_input > 100 && *brake_input <= 200){
		*brake_output = 2;
	}
	else if(*brake_input > 200 && *brake_input <= 300){
		*brake_output = 3;
	}
	else if(*brake_input > 300 && *brake_input <= 400){
		*brake_output = 4;
	}
	else if(*brake_input > 400 && *brake_input <= 500){
		*brake_output = 5;
	}
}
