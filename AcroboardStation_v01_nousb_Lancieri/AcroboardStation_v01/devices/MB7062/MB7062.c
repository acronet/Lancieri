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

#include <asf.h>
#include <stdio.h>
#include <string.h>
#include "board.h"
//#include "sysclk.h"

#include "conf_board.h"

#include "drivers/usart_interrupt/buffer_usart.h"
#include "config/conf_usart_serial.h"
#include "globals.h"

#include "devices/MB7062/MB7062.h"

// SENSORCODE
#define LG_EQUALMEASURES		3	// Number of measures with the same value we want to obtain 
#define LG_STRINGBUFSIZE		5	// Is the length of the string buffer
#define LG_BUFSIZE				24	// Number of measures involved in the getStats function
#define LG_SAMPLEMEASUES		21	// Number of reading involved in the saveSample function

static uint16_t g_measureBuffer[LG_BUFSIZE], g_measureInstantBuf[LG_SAMPLEMEASUES];
static char g_LGszBuf[LG_STRINGBUFSIZE],g_LGstring[LG_STRINGBUFSIZE];
static uint8_t g_LGrecordingData = false, g_index = 0, g_LGmeasureCounter = 0, g_measureOverflow = 0;

static uint16_t MB7062_adcGetValue( void );
static void MB7062_interrupt_start(void);
static void MB7062_interrupt_stop(void);
static void MB7062_saveSample(void);
static bool usartc_USARTRS232LG_1_RX_CBuffer_Complete(void);

void MB7062_init(void)
{
	//USART options.
	static usart_rs232_options_t RS232_SERIAL_OPTIONS = {
		.baudrate = USART_RS232_1_BAUDRATE,
		.charlength = USART_CHAR_LENGTH,
		.paritytype = USART_PARITY,
		.stopbits = USART_STOP_BIT
	};
	sysclk_enable_module(SYSCLK_PORT_C,PR_USART0_bm);
	usart_serial_init(USART_RS232_1, &RS232_SERIAL_OPTIONS);
	usart_set_rx_interrupt_level(USART_RS232_1,USART_INT_LVL_LO);	
	//Configure the switch and set at low level
	ioport_configure_pin(LEVELGAUGE_SWITCH, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LEVELGAUGE_SWITCH , LOW);
	
	MB7062_resetStats();
	
	//Configure the ADC Port
	// ToDo
	
	//SENSORCODE: Disable JTag wheh using PORTB gpio Pins
	// DISABLE jtag - it locks the upper 4 pins of PORT B
	//CCP       = CCP_IOREG_gc;    // Secret handshake
	//MCU.MCUCR = 0b00000001;
	
	sei();
}

void MB7062_triggerReading(void)
{
	//Enable switch	ioport_set_pin_level(LEVELGAUGE_SWITCH , HIGH);
	MB7062_interrupt_start();
}

void MB7062_getStats(LG_MB7062_STATS * const ps)
{
	LG_MB7062_STATS measureBuf;
	char szBuf[128];
	uint32_t partialsum = 0;
	uint16_t num = 0, tempVal;
	while (g_LGrecordingData);
	if(g_measureOverflow)
		num = LG_BUFSIZE;
	else
		num = g_index;
	
	// Sort the measures array
	for (uint8_t i = 0 ; i < num ; i++)
	{
		for (uint8_t j = i+1 ; j < num ; j++)
		{
			if (g_measureBuffer[i] > g_measureBuffer[j])
			{
				tempVal =  g_measureBuffer[i] ;
				g_measureBuffer[i]  = g_measureBuffer[j];
				g_measureBuffer[j] = tempVal;
			}
		}
	}
	
	//
	for (int i = 0 ; i < num ; i++)
	{
		partialsum += g_measureBuffer[i];
		sprintf_P(szBuf,PSTR("val: %u - partsum: %lu\r\n"),g_measureBuffer[i],partialsum);
		debug_string(VERBOSE,szBuf,false);
	}
	
	measureBuf.val = partialsum / num;
	measureBuf.maxVal = g_measureBuffer[ (num-1) ];
	measureBuf.minVal = g_measureBuffer[0];
	measureBuf.medianVal = g_measureBuffer[ (num/2) ]; //num is the number of elements in the array
	measureBuf.val_noPeaks = (partialsum - measureBuf.maxVal - measureBuf.minVal) / (num - 2) ;
	
	//measureBuf.adcVal=MB7062_adcGetValue();

	sprintf_P(szBuf,PSTR("val: %u - medianVal = %u - maxVal: %u - minVal: %u - Val_noPeaks: %u\r\n"),measureBuf.val,measureBuf.medianVal,measureBuf.maxVal,measureBuf.minVal,measureBuf.val_noPeaks);
	debug_string(NORMAL,szBuf,false);
	memcpy_ram2ram(ps,&measureBuf,sizeof(LG_MB7062_STATS));
	MB7062_resetStats();
}

void MB7062_resetStats(void)
{
	memset(g_measureBuffer, 0, LG_BUFSIZE * sizeof(g_measureBuffer[0]));
	memset(g_measureInstantBuf, 0, LG_SAMPLEMEASUES * sizeof(g_measureBuffer[0]));
	g_LGmeasureCounter = 0;
	g_index = 0;
	g_measureOverflow = 0;
}

static uint16_t MB7062_adcGetValue( void )
{
	//ToDO
	return 0;
}

static void MB7062_interrupt_start(void){	g_LGszBuf[0]= 'R';	g_LGszBuf[1]= '9';	g_LGszBuf[2]= '9';	g_LGszBuf[3]= '9';	g_LGszBuf[4]= 0;	g_LGstring[0]= 'R';	g_LGstring[1]= '9';	g_LGstring[2]= '9';	g_LGstring[3]= '9';	g_LGstring[4]= 0;	//Enable interrupt	usart_set_rx_interrupt_level(USART_RS232_1,USART_INT_LVL_LO);  //Occhio alla priorità	usart_rx_enable(USART_RS232_1);}
static void MB7062_interrupt_stop(void){	//Disable switch	ioport_set_pin_level(LEVELGAUGE_SWITCH , LOW);	//Disable interrupt	usart_set_rx_interrupt_level(USART_RS232_1,USART_INT_LVL_OFF);	usart_rx_disable(USART_RS232_1);}

static void MB7062_saveSample(void)
{
	uint16_t reading = 0, tempVal = 0;
	char szBuf[128];
	g_LGrecordingData = true;
	reading += (uint16_t)(g_LGstring[2] - 48);
	reading += (uint16_t)(g_LGstring[1]-48) * 10 ;
	reading += (uint16_t)(g_LGstring[0]-48) * 100 ;
	g_measureInstantBuf[g_LGmeasureCounter++] = reading;
	//sprintf_P(szBuf,PSTR("S: %s - C: %u\r\n"),g_LGstring,g_LGmeasureCounter);
	//debug_string(NORMAL,szBuf,false);
	if (g_LGmeasureCounter >= LG_SAMPLEMEASUES)
	{
		MB7062_interrupt_stop();
		
		// Sort the measures array
		for (uint8_t i = 0 ; i < LG_SAMPLEMEASUES ; i++)
		{
			for (uint8_t j = i+1 ; j < LG_SAMPLEMEASUES ; j++)
			{
				if (g_measureInstantBuf[i] > g_measureInstantBuf[j])
				{
					tempVal =  g_measureInstantBuf[i] ;
					g_measureInstantBuf[i]  = g_measureInstantBuf[j];
					g_measureInstantBuf[j] = tempVal;
				}
			}
		}
		
		if(g_index <= LG_BUFSIZE-1)
			g_measureBuffer[g_index++] = g_measureInstantBuf[LG_SAMPLEMEASUES/2];
		else
		{
			g_index = 0;
			g_measureBuffer[g_index++] = g_measureInstantBuf[LG_SAMPLEMEASUES/2];
			g_measureOverflow = 1;
		}
		sprintf_P(szBuf,PSTR("I: %u - Val: %u - index = %u\r\n"),g_index-1,g_measureBuffer[g_index-1],LG_SAMPLEMEASUES/2);
		debug_string(NORMAL,szBuf,false);
		g_LGmeasureCounter = 0;
	}
	
	
	
	g_LGrecordingData = false;
}



static bool usartc_USARTRS232LG_1_RX_CBuffer_Complete(void)
{
	USART_t * const ad = USART_RS232_1;
	static uint8_t idx=0;	const uint8_t dt = ad->DATA;
	
	//char szBUF[2];
	//szBUF[0]=dt;
	//szBUF[1]=0;
	//debug_string(VERBOSE,szBUF,false);
		/* Advance buffer head. */	if (idx==0)	{		if (dt == 'R')			g_LGszBuf[idx++]=dt;		else			idx=0;	}	else		g_LGszBuf[idx++]=dt;			if ( idx == LG_STRINGBUFSIZE)	{		idx=0;		g_LGstring[0]= g_LGszBuf[1];		g_LGstring[1]= g_LGszBuf[2];		g_LGstring[2]= g_LGszBuf[3];		g_LGstring[3]= 0;		MB7062_saveSample();	}	return true;
}

ISR(USARTC0_RXC_vect)
{
	usartc_USARTRS232LG_1_RX_CBuffer_Complete();
}