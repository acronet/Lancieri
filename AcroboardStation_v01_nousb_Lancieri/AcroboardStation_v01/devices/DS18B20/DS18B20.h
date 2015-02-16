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


#ifndef DS18B20_H_
#define DS18B20_H_

//DS18B0 connections
#define DS18B20_COM_PORT			IOPORT_CREATE_PIN(PORTA, 3)

#define DS18B20_FAMILY_ID			0x28
//1-wire ROM commands
#define DS18B20_CMD_READ_ROM		0x33
#define DS18B20_CMD_MATCH_ROM		0x55
#define DS18B20_CMD_SEARCH_ROM		0xF0
#define DS18B20_CMD_ALARM_SEARCH	0xEC
#define DS18B20_CMD_SKIP_ROM		0xCC
//DS18B20 functional commands
#define DS18B20_CONVERT_T			0x44
#define DS18B20_READ_SCRATCHPAD		0xBE
#define DS18B20_WRITE_SCRATCHPAD	0x4E
#define DS18B20_COPY_SCRATCHPAD		0x48
#define DS18B20_RECALL_EEPROM		0xB8
#define DS18B20_READ_POWERSUPPLY	0xB4

void DS18B20_get_stats(int16_t * const ps);
void DS18B20_getValue(void);

#endif /* DS18B20_H_ */