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


#ifndef DS18B20_2_H_
#define DS18B20_2_H_

//DS18B0 connections
#define DS18B20_2_COM_PORT			IOPORT_CREATE_PIN(PORTA, 5)
void DS18B20_2_get_stats( int16_t * ps );

void DS18B20_2_getValue(void);
#endif /* DS18B20_H_ */