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

#include "devices/vegapuls61/vegapuls61.h"

static uint16_t vp61_voltageToLevelConverter( uint16_t );
static uint8_t ReadCalibrationBytes( uint8_t );
//static uint16_t vp61_getAnalogValue( void );
static uint16_t vp61_getAnalogValue(uint16_t , float );
static uint16_t vp61_getOffset( void );
static float vp61_getGain( uint16_t );
#define VP61_ADC_OVERSAMPLING 128

#define MEASUREBUFFERSIZE		256
#define LG_BUFSIZE				23 // Number of reading involved in the getStats function


static uint16_t g_measureBuffer[MEASUREBUFFERSIZE];
static uint16_t g_index = 0;


#ifdef BOARD_ACROSTATION_R08
#define ADC_SCALE_FACTOR			0.50354F
#define ADC_DAC_BANDGAP_VALUE		2185
#endif

#ifdef BOARD_ACROSTATION_R09
#define ADC_SCALE_FACTOR			0.42725F
#define ADC_DAC_BANDGAP_VALUE		2575
#endif


static uint8_t ReadCalibrationBytes( uint8_t index )
{
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return( result );
}


//static uint16_t vp61_getAnalogValue( void )
static uint16_t vp61_getAnalogValue(uint16_t offset, float gain )
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	uint8_t inputgain = 1;
	char szBUF[64];
	uint8_t ii = 0;
	uint16_t measure;
	uint32_t result = 0;
	
	adc_read_configuration(&VP61_ADC, &adc_conf);
	adcch_read_configuration(&VP61_ADC, VP61_ADC_CH, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_FREERUN, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);

	adcch_set_input(&adcch_conf, VP61_PIN, ADCCH_NEG_NONE, inputgain);
	adcch_write_configuration(&VP61_ADC, VP61_ADC_CH, &adcch_conf);
	adc_write_configuration(&VP61_ADC, &adc_conf);

	VP61_ADC_PIN = PORT_OPC_TOTEM_gc;
	delay_ms(2);
	
	adc_enable(&VP61_ADC);
	
	for (uint8_t jj = 0 ; jj == 10 ; jj++)
		adc_wait_for_interrupt_flag(&VP61_ADC, VP61_ADC_CH);
	
	while(1)
	{
		adc_wait_for_interrupt_flag(&VP61_ADC, VP61_ADC_CH);
		measure = ( adc_get_result(&VP61_ADC, VP61_ADC_CH) - offset ) ;
		result += measure;
		sprintf_P(szBUF,PSTR("%u\t\t%lu\r\n"),measure,result);
		debug_string(VERBOSE,szBUF,RAM_STRING);
		ii++;
		if (ii>=VP61_ADC_OVERSAMPLING)
		{
			adc_disable(&VP61_ADC);
			break;
		}
	}
	
	result /= VP61_ADC_OVERSAMPLING; 
	
	
	//VP61_ADC_PIN = PORT_OPC_PULLUP_gc;
	
	sprintf_P(szBUF,PSTR("VP61 ADC: %d\r\n"),result);
	debug_string(NORMAL,szBUF,RAM_STRING);
	
	const uint16_t voltage =  ( (float)result * ADC_SCALE_FACTOR * gain ) ;
	
	sprintf_P(szBUF,PSTR("VP61 Voltage: %d\r\n"),voltage);
	debug_string(NORMAL,szBUF,RAM_STRING);
	
	return voltage;
	
}

static uint16_t vp61_voltageToLevelConverter(uint16_t adcVoltage)
{
	const uint16_t level = (float)adcVoltage * 1.1667F - 482.962F; //1.166F - 4.830F is the linear calibration
	return level;
}

static uint16_t vp61_getOffset( void )
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	uint8_t inputgain = 1;
	char szBUF[64];
	uint8_t ii = 0;
	uint32_t offset = 0;
	
	adc_read_configuration(&VP61_ADC, &adc_conf);
	adcch_read_configuration(&VP61_ADC, VP61_ADC_CH, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_FREERUN, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);

	adcch_set_input(&adcch_conf, VP61_GND, ADCCH_NEG_NONE, inputgain);
	adcch_write_configuration(&VP61_ADC, VP61_ADC_CH, &adcch_conf);
	adc_write_configuration(&VP61_ADC, &adc_conf);
	
	VP61_ADC_GND = PORT_OPC_TOTEM_gc;
	delay_ms(2);
	
	adc_enable(&VP61_ADC);
	
	for (uint8_t jj = 0 ; jj == 10 ; jj++)
	adc_wait_for_interrupt_flag(&VP61_ADC, VP61_ADC_CH);
	
	while(1)
	{
		adc_wait_for_interrupt_flag(&VP61_ADC, VP61_ADC_CH);
		offset += adc_get_result(&VP61_ADC, VP61_ADC_CH);
		ii++;
		if (ii>=VP61_ADC_OVERSAMPLING)
		{
			adc_disable(&VP61_ADC);
			break;
		}
	}
	
	offset /= VP61_ADC_OVERSAMPLING;
	
	
	//VP61_ADC_GND = PORT_OPC_PULLUP_gc;
	
	sprintf_P(szBUF,PSTR("VP61 Offset: %lu\r\n"),offset);
	debug_string(VERBOSE,szBUF,RAM_STRING);
	
	return offset;
}

static float vp61_getGain (uint16_t offset )
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	uint8_t inputgain = 1;
	char szBUF[64];
	uint8_t ii = 0;
	uint32_t val = 0;
	
	adc_read_configuration(&VP61_ADC, &adc_conf);
	adcch_read_configuration(&VP61_ADC, VP61_ADC_CH, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_FREERUN, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);

	adcch_set_input(&adcch_conf, ADCCH_POS_BANDGAP, ADCCH_NEG_NONE, inputgain);
	adcch_write_configuration(&VP61_ADC, VP61_ADC_CH, &adcch_conf);
	adc_write_configuration(&VP61_ADC, &adc_conf);
	
	adc_enable(&VP61_ADC);
	
	for (uint8_t jj = 0 ; jj == 10 ; jj++)
		adc_wait_for_interrupt_flag(&VP61_ADC, VP61_ADC_CH);
	
	while(1)
	{
		adc_wait_for_interrupt_flag(&VP61_ADC, VP61_ADC_CH);
		val += ( adc_get_result(&VP61_ADC, VP61_ADC_CH) - offset);
		ii++;
		if (ii>=VP61_ADC_OVERSAMPLING)
		{
			adc_disable(&VP61_ADC);
			break;
		}
	}
	
	val /= VP61_ADC_OVERSAMPLING;
	
	const float gain = (float)ADC_DAC_BANDGAP_VALUE / (float)val;
	
	sprintf_P(szBUF,PSTR("VP61 Gain: %u\r\n"),(uint16_t)gain*100);
	debug_string(VERBOSE,szBUF,RAM_STRING);
	
	return gain;
}

void vp61_init(void)
{
	ADCA.CALL = ReadCalibrationBytes( ADCACAL0 );
	ADCA.CALH = ReadCalibrationBytes( ADCACAL1 );
	// Set all g_measureBuffer values to zero
	vp61_resetStats();
}

void vp61_triggerReading( void )
{
	const uint16_t offset = vp61_getOffset();
	const float gain = vp61_getGain(offset);
	const uint16_t reading = vp61_getAnalogValue(offset,gain);
	
	g_measureBuffer[g_index++]=reading;
}

void vp61_getStats(LG_VP61_STATS * const ps)
{
	char szBUF[128];
	LG_VP61_STATS measureBuf;
	uint32_t partialSum = 0;
	//vp61_triggerReading();
	
	uint16_t start_index;
	if (g_index - LG_BUFSIZE >=0)	//reading every 5 second --> 24 readings = 2 min
		start_index = g_index - LG_BUFSIZE;
	else
		start_index = 0;

	sprintf_P(szBUF,PSTR("VP61 g_index: %d\r\n"),g_index);
	debug_string(VERBOSE,szBUF,RAM_STRING);
		
	uint16_t tempVal;
	for (uint8_t i = start_index ; i < g_index ; i++)
	{
		for (uint8_t j = i+1 ; j < g_index ; j++)
		{
			if (g_measureBuffer[i] > g_measureBuffer[j])
			{
				tempVal =  g_measureBuffer[i] ;
				g_measureBuffer[i]  = g_measureBuffer[j];
				g_measureBuffer[j] = tempVal;
			}
		}
		partialSum += g_measureBuffer[i];
		sprintf_P(szBUF,PSTR("%u\t%lu\r\n"),g_measureBuffer[i],partialSum);
		debug_string(NORMAL,szBUF,RAM_STRING);
	}
	measureBuf.voltageVal = partialSum / (g_index - start_index);
	measureBuf.maxVal = g_measureBuffer[g_index-1];
	measureBuf.minVal = g_measureBuffer[start_index];
	measureBuf.voltageValNoPeaks = ( partialSum - measureBuf.maxVal - measureBuf.minVal ) / (g_index - start_index - 2);
	measureBuf.medianVal = g_measureBuffer[ start_index + (g_index - start_index) / 2];
	
	sprintf_P(szBUF,PSTR("VP61 - V: %u\tVnp: %u\tVmed: %u\tVmax: %u\tVmin: %u\r\n"),measureBuf.voltageVal,measureBuf.voltageValNoPeaks,measureBuf.medianVal,measureBuf.maxVal,measureBuf.minVal);
	debug_string(NORMAL,szBUF,RAM_STRING);
	
	sprintf_P(szBUF,PSTR("Indexes - Start: %u\tStop = %u\tmed: %u\r\n"),start_index,g_index-1,start_index + (g_index - start_index) / 2);
	debug_string(NORMAL,szBUF,RAM_STRING);
	
	//Da Verificare//Distance in cm
	//measureBuf.distVal = vp61_voltageToLevelConverter(measureBuf.voltageVal);
	//sprintf_P(szBUF,PSTR("VP61 level=: %d\r\n"),measureBuf.distVal);
	//debug_string(VERBOSE,szBUF,RAM_STRING);
	memcpy_ram2ram(ps,&measureBuf,sizeof(LG_VP61_STATS));
	vp61_resetStats();
}

void vp61_resetStats(void)
{
	memset(g_measureBuffer, 0, MEASUREBUFFERSIZE * sizeof(g_measureBuffer[0]));
	g_index = 0;
}