#ifndef _TB6612FNG_H
#define _TB6612FNG_H

#include "stdbool.h"
// I am using two drivers, one for the two motors on the front and the other for the two in the rear.

/* Motor driver typedef struct to contain all necessary hardware peripherals in one place.
 * Both motor drivers are included in the same struct.
*/
typedef struct{
	// Standby pins
	GPIO_TypeDef* stdby_front_port;
	uint16_t stdby_front_pin;

	GPIO_TypeDef* stdby_rear_port;
	uint16_t stdby_rear_pin;

	/* IN1 and IN2 pins for each motor. These pins determine direction of motor rotation.
	 * Each motor has two pairs of IN1 and IN2 pins denoted by A and B.
	*/

	// Front left
	GPIO_TypeDef* AIN1_front_port;
	uint16_t AIN1_front_pin;

	GPIO_TypeDef* AIN2_front_port;
	uint16_t AIN2_front_pin;

	// Front right
	GPIO_TypeDef* BIN1_front_port;
	uint16_t BIN1_front_pin;

	GPIO_TypeDef* BIN2_front_port;
	uint16_t BIN2_front_pin;

	// Rear left
	GPIO_TypeDef* AIN1_rear_port;
	uint16_t AIN1_rear_pin;

	GPIO_TypeDef* AIN2_rear_port;
	uint16_t AIN2_rear_pin;

	// Rear right
	GPIO_TypeDef* BIN1_rear_port;
	uint16_t BIN1_rear_pin;

	GPIO_TypeDef* BIN2_rear_port;
	uint16_t BIN2_rear_pin;

	// Using the same timer peripheral for generating PWM for all four motors.
	TIM_HandleTypeDef* htim;

	// Timer channels
	uint32_t front_left;
	uint32_t front_right;
	uint32_t rear_left;
	uint32_t rear_right;

}tb6612fng_t;

// Function declarations
void tb6612fng_init(tb6612fng_t* tbf);
void tb6612fng_direction(tb6612fng_t* tbf, bool in1, bool in2);
void tb6612fng_speed(tb6612fng_t* tbf ,uint16_t speed,float sideA,float sideB);

#endif
