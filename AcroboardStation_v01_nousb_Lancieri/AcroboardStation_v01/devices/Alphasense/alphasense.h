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


#ifndef ALPHASENSE_H_
#define ALPHASENSE_H_

#define ALPHASENSE_ADC			ADCB
#define ALPHASENSE_ADC_CH		ADC_CH0
#define ALPHASENSE_ID			0

typedef struct
{
	int16_t		working;
	int16_t		aux;
} ALPHASENSE_STATS;

void alphasense_get_stats( ALPHASENSE_STATS * const ps );
void alphasense_reset_stats( void );
void alphasense_init( void );
void alphasense_adc_getValue( void );

#endif /* ALPHASENSE_H_ */