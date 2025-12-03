/*
 * EMA.h
 *
 *  Created on: Oct 8, 2025
 *      Author: manas
 */

#ifndef INC_EMA_H_
#define INC_EMA_H_

typedef struct{
	float alpha;
	float out;
}EMA_t;

void EMA_init(EMA_t *filter, float alpha);
void EMA_setAlpha(EMA_t *filter, float alpha);
float EMA_update(EMA_t *filter, float input);

#endif /* INC_EMA_H_ */
