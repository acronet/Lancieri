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

#ifndef SENS_IT_H_
#define SENS_IT_H_

typedef struct {
	int32_t gas[3];				// gas measurement CO - NO2 - O3
	//uint32_t res;				// resistence
	//int16_t temp;				// temperature
	//int16_t fan;				// fan (RPS)
	int16_t error_check[3];	// check for errors: error if != 0 CO - NO2 - O3
	//uint8_t target_gas [5];		// the first character is the sensor address (A-Z) and the second
								// specifies the target gas: 1=CO - 2=NO2 - 3=O3 - 4=CH4 - 5=NOx - 6=C6H6
	uint16_t readingNumber[3];
} SENSIT_STATS;

void sens_it_init(void);
void sens_it_triggerReading(uint8_t address);
void sens_it_getStats(SENSIT_STATS * const);
void sens_it_resetStats(void);

#endif /* SENS_IT_H_ */