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


#ifndef VEGAPULS61_H_
#define VEGAPULS61_H_

////////////////////////////////////////////////////////////////////
// SENSORCODE: Define here the ADC Port and Pinout
#define VP61_ID				0
#define VP61_ADC_PORT		PORTA
#define VP61_ADC			ADCA
#define VP61_ADC_CH			ADC_CH0
#define VP61_ADC_PIN		VP61_ADC_PORT.PIN3CTRL

#define VP61_ADC_OFF_PORT	PORTA
#define VP61_ADC_OFF		ADCA
#define VP61_ADC_OFF_CH		ADC_CH1
#define VP61_ADC_GND		VP61_ADC_PORT.PIN5CTRL
////////////////////////////////////////////////////////////////////

typedef struct {
	//uint16_t distVal;
	uint16_t voltageVal;
	uint16_t maxVal;
	uint16_t minVal;
	uint16_t voltageValNoPeaks;
	uint16_t medianVal;
} LG_VP61_STATS;

void vp61_init(void);
void vp61_triggerReading(void);
void vp61_getStats(LG_VP61_STATS * const);
void vp61_resetStats(void);

#endif /* VEGAPULS61_H_ */