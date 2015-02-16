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


#ifndef LEVELGAUGE_H_
#define LEVELGAUGE_H_

#define LG_ADC_PORT		PORTB
#define LG_ADC			ADCB
#define LG_ADC_CH		ADC_CH0
#define LG_ID			0

typedef struct {
	uint16_t val;
	uint16_t val_noPeaks;
	uint16_t adcVal;
	uint16_t maxVal;
	uint16_t minVal;
	uint16_t medianVal;
} LG_MB7062_STATS;

void MB7062_init(void);
void MB7062_triggerReading(void);
void MB7062_getStats(LG_MB7062_STATS * const );
void MB7062_resetStats(void);

#endif /* LEVELGAUGE_H_ */