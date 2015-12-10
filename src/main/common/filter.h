/*
 * filter.h
 *
 *  Created on: 24 jun. 2015
 *      Author: borisb
 */



typedef struct filterStatePt1_s {
	float state;
	float RC;
} filterStatePt1_t;

float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dt);
int8_t * filterGetFIRCoefficientsTable(void);
void filterApplyFIR(int16_t data[3], int16_t state[3][7], int8_t coeff[7]);
