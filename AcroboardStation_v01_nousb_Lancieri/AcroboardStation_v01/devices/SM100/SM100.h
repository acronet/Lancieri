/*
 * ACRONET Project
 * http://www.acronet.cc
 *
 * Copyright ( C ) 2014 Acrotec srl
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the EUPL v.1.1 license.  See http://ec.europa.eu/idabc/eupl.html for details.
 */ 


#ifndef SM100_H_
#define SM100_H_

#define SM100_ADC					ADCB
#define SM100_ADC_CH				ADC_CH0
#define SM100_ADC_REF				ADC_REF_AREFB
#define SM100_ADC_SENSOR_B			ADCCH_POS_PIN2
#define SM100_ADC_SENSOR_A			ADCCH_POS_PIN4
#define SM100_ADC_GND				ADCCH_POS_PIN5

typedef struct
{
	int16_t		sensorA;
	int16_t		sensorB;
	int16_t		lastA;
	int16_t		lastB;
	int16_t		ratioA;
	int16_t		ratioB;
	int16_t		counter;
} SM100_STATS;

void SM100_get_stats( SM100_STATS * const ps );
void SM100_getValue(void);
static void SM100_start(void);
void SM100_init(void);


#endif /* SM100_H_ */