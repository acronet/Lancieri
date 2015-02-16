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


#ifndef PM10QBIT_H_
#define PM10QBIT_H_

typedef struct {
	// remember to initialize attributes
	// PM10QBIT_STATS PM10val = {.mean = 0.0f, .samples = 0, .maxVal = 0, .minVal = 0};
	uint8_t samples;
	uint16_t mean;
	//uint16_t maxVal;
	//uint16_t minVal;
	uint8_t valDistr[20];
} PM10QBIT_STATS;

void PM10Qbit_init(void);

void PM10Qbit_get_value (PM10QBIT_STATS * const);

#endif /* PM10QBIT_H_ */