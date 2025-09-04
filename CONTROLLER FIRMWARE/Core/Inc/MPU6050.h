/*
 * MPU6050.h
 *
 *  Created on: Jul 30, 2025
 *      Author: manas
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define MPU6050_ADDR 0xD0 // The default 7-bit address is 0x68. It is left shifted as suggested in the HAL driver manual(I2C specific)

#define WHO_AM_I 		0x75
#define PWR_MGMT_1  	0x68
#define SMPRT_DIV   	0x19
#define GYRO_CONFIG 	0x1B
#define ACCEL_CONFIG 	0x1C
#define GYRO_XOUT		0x43
#define ACCEL_XOUT_H    0x3B

typedef struct{
	float gyro_x;
	float gyro_y;
	float gyro_z;

	float accel_x;
	float accel_y;
	float accel_z;
}MPU6050_t;

void MPU6050_init();
void MPU6050_Read_Gyro(MPU6050_t* mpu);
void MPU6050_Read_Accel(MPU6050_t* mpu);

#endif /* INC_MPU6050_H_ */



