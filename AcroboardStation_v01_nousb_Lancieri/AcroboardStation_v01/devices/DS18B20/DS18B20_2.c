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
#include <stdint.h>

#include <stdio.h>
#include <delay.h>
//#define F_CPU 2000000
#include <util/delay.h>
#include <asf.h>
#include "board.h"
#include "globals.h"

#include "conf_board.h"
#include "DS18B20_2.h"
#include "DS18B20.h"
#include <util/crc16.h>

#define DS18B20_2_OUTPUT	ioport_configure_pin(DS18B20_2_COM_PORT, IOPORT_DIR_OUTPUT);
#define DS18B20_2_INPUT		ioport_configure_pin(DS18B20_2_COM_PORT, IOPORT_DIR_INPUT);

#define THERM_DECIMAL_STEPS_12BIT	625	//.0625
#define THERM_DECIMAL_STEPS_11BIT	125	//.125
#define THERM_DECIMAL_STEPS_10BIT	25	//.250
#define THERM_DECIMAL_STEPS_9BIT	5		//.5

//uint16_t g_stats;
int32_t g_partialSumT2 = 0;
uint16_t g_partialCounterT2 = 0;
volatile bool g_recordigTemp2=false;

//Print on the debug port a binary representation of a byte 
static void bin(unsigned n)
{
	unsigned i;
	for (i = 1 << 7; i > 0; i = i / 2)
	(n & i)? debug_string(NORMAL,PSTR("1"),NORMAL) : debug_string(NORMAL,PSTR("0"),true);
	debug_string(NORMAL,PSTR("\r\n"),true);
}

static bool DS18B20_2_reset(void){
	
	bool presence = false;
	//Reset signal
	DS18B20_2_OUTPUT;
	gpio_set_pin_low(DS18B20_2_COM_PORT);
	_delay_us(480);
	
	//Release line and wait and then save the vaule
	gpio_set_pin_high(DS18B20_2_COM_PORT);
	DS18B20_2_INPUT;
	_delay_us(60);
	presence = ioport_get_value(DS18B20_2_COM_PORT);
	_delay_us(420);	//480-60
	return !presence;
}

static void DS18B20_2_write_bit(uint8_t bit){
	//Pull line low for 1uS
	DS18B20_2_OUTPUT;
	gpio_set_pin_low(DS18B20_2_COM_PORT);
	_delay_us(2);
	//Write 1
	if(bit) gpio_set_pin_high(DS18B20_2_COM_PORT);
	//Write 0
	_delay_us(60);
	gpio_set_pin_high(DS18B20_2_COM_PORT);
}

static bool DS18B20_2_read_bit(void){
	bool bit=0;
	//Pull line low for 1uS
	DS18B20_2_OUTPUT;
	gpio_set_pin_low(DS18B20_2_COM_PORT);
	_delay_us(3);
	//Release line and wait for 14uS
	DS18B20_2_INPUT;
	_delay_us(9);
	//Read line value
	bit = ioport_get_value(DS18B20_2_COM_PORT);
	_delay_us(45);
	return bit;
}


static uint8_t DS18B20_2_read_byte(void){
	cli();
	uint8_t i=8, byte=0;
	while(i--){
		byte>>=1;
		byte|=(DS18B20_2_read_bit()<<7);
		
	}
	sei();
	return byte;
}

static void DS18B20_2_write_byte(uint8_t byte){
	uint8_t i=8;		
	while(i--){
		DS18B20_2_write_bit(byte&1);
		byte>>=1;
	}
}

void DS18B20_2_get_stats(int16_t * const ps){
	//*ps = g_stats;
	while(g_recordigTemp2);
	*ps = (float)g_partialSumT2 / (float)g_partialCounterT2;
	g_partialSumT2 = 0;
	g_partialCounterT2 = 0;
}

void DS18B20_2_getValue( void )
{
	int16_t result;
	DS18B20_2_OUTPUT;
	PORTA.PIN4CTRL = PORT_OPC_TOTEM_gc;
	bool negative = false;
	uint8_t scretchpad[9];
	uint16_t dec;
	uint16_t dig;
	//char szBuf[32];
	//sprintf(szBuf,"meas %d\r\n",c++);
	//debug_string(NORMAL,szBuf,false);
	
	//Since we have only one 1-wire device we can skip ROM reading and start temperature conversion
	DS18B20_2_reset();
	//if(prova)
		//debug_string(NORMAL,PSTR("Sensor present\r\n"),true);
	//else
		//debug_string(NORMAL,PSTR("Missing sensor\r\n"),true);
	DS18B20_2_write_byte(DS18B20_CMD_SKIP_ROM);
	DS18B20_2_write_byte(DS18B20_CONVERT_T);
	//while(gpio_pin_is_low(DS18B20_2_COM_PORT));	//wait until convrsion is done
	delay_ms(750);
	//debug_string(NORMAL,PSTR("Conversion finisehd\r\n"),true);
	DS18B20_2_reset();
	DS18B20_2_write_byte(DS18B20_CMD_SKIP_ROM);
	DS18B20_2_write_byte(DS18B20_READ_SCRATCHPAD);
	
	
	
	for (int i = 0 ; i < 9 ; i++)
		scretchpad[i] = DS18B20_2_read_byte();
	DS18B20_2_reset();
	PORTA.PIN4CTRL = PORT_OPC_PULLUP_gc;
	uint8_t crcByte = 0x00;
	//for (int i = 0 ; i < 9 ; i++){
		//bin(scretchpad[i]);
		//if(i<8)
			//crcByte = _crc_ibutton_update(crcByte, scretchpad[i]);
	//}
	//debug_string(NORMAL,PSTR("\r\ncrc = "),true);
	//bin(crcByte);
	//debug_string(NORMAL,PSTR("\r\n"),true);
	
	for (int i = 0 ; i < 8 ; i++)
		crcByte = _crc_ibutton_update(crcByte, scretchpad[i]);
		
	if (crcByte == scretchpad[8]){
		if (scretchpad[1]&0x80)
			negative = true;
	
		dec = scretchpad[0] >> 4;
		dec |= (scretchpad[1]&0x7) << 4;
	
		dig = scretchpad[0]&0xf;
		dig*=THERM_DECIMAL_STEPS_12BIT;
		if (!negative)
		result = (int16_t)dec*100 +  (int16_t)dig / 100;
		else
		result =  -1 *  (int16_t)dec*100 +  (int16_t)dig / 100;
	}
	else
	{
		result =  -9999;
		// debug_string(NORMAL,PSTR("\r\nT: -9999"),true);
		// return;
	}
	g_recordigTemp2 = true;
	g_partialSumT2+=result;
	g_partialCounterT2++;
	g_recordigTemp2 = false;
	
	char szBuf[32];
	sprintf(szBuf,"2: sample: %u\tT: %d\tpartialSum: %ld\r\n",g_partialCounterT2,result,g_partialSumT2);
	debug_string(NORMAL,szBuf,false);
	
	//g_stats =  result;
}