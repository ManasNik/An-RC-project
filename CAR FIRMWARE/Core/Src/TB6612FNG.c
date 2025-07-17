#include "main.h"
#include "TB6612FNG.h"

// Function for motor driver initialization
void tb6612fng_init(tb6612fng_t* tbf){
	// Initialization of standby pins
	HAL_GPIO_WritePin(tbf->stdby_front_port, tbf->stdby_front_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(tbf->stdby_rear_port, tbf->stdby_rear_pin, GPIO_PIN_SET);

	// Initialization of PWM channels for each motor
	HAL_TIM_PWM_Start(tbf->htim,tbf->front_left);
	HAL_TIM_PWM_Start(tbf->htim,tbf->front_right);
	HAL_TIM_PWM_Start(tbf->htim,tbf->rear_left);
	HAL_TIM_PWM_Start(tbf->htim,tbf->rear_right);
}

// Function for changing the direction between forward and reverse
void tb6612fng_direction(tb6612fng_t* tbf, bool in1, bool in2){
	// Left side motors
	HAL_GPIO_WritePin(tbf->AIN1_front_port, tbf->AIN1_front_pin, in1);
	HAL_GPIO_WritePin(tbf->AIN2_front_port, tbf->AIN2_front_pin, in2);

	HAL_GPIO_WritePin(tbf->AIN1_rear_port, tbf->AIN1_rear_pin, in1);
	HAL_GPIO_WritePin(tbf->AIN2_rear_port, tbf->AIN2_rear_pin, in2);

	// Right side motors
	HAL_GPIO_WritePin(tbf->BIN1_front_port, tbf->BIN1_front_pin, in1);
	HAL_GPIO_WritePin(tbf->BIN2_front_port, tbf->BIN2_front_pin, in2);

	HAL_GPIO_WritePin(tbf->BIN1_rear_port, tbf->BIN1_rear_pin, in1);
	HAL_GPIO_WritePin(tbf->BIN2_rear_port, tbf->BIN2_rear_pin, in2);

}

// Function for changing motor speed using PWM
void tb6612fng_speed(tb6612fng_t tbf ,uint16_t speed_left, uint16_t speed_right){
	// Applying the speed to all the four motors
	__HAL_TIM_SET_COMPARE(tbf.htim,tbf.front_left,speed_left);
	__HAL_TIM_SET_COMPARE(tbf.htim,tbf.front_right,speed_right);
	__HAL_TIM_SET_COMPARE(tbf.htim,tbf.rear_left,speed_left);
	__HAL_TIM_SET_COMPARE(tbf.htim,tbf.rear_right,speed_right);

}
