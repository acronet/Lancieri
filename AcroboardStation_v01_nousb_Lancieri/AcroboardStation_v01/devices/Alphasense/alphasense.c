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
#include "globals.h"
#include "devices/Alphasense/alphasense.h"
#include "devices/statusled/status_led.h"
#include <math.h>
#include <stdio.h>

static ALPHASENSE_STATS g_internal;

int32_t g_partialSumWork = 0, g_partialSumAux = 0;
uint16_t g_partialCount_Alphasense = 0;
volatile bool g_recordingData_Alphasense = false;

void alphasense_get_stats( ALPHASENSE_STATS * const ps )
{
	while(g_recordingData_Alphasense);
	//meglio assegnare direttamente i due parametri?
	g_internal.working = (float)g_partialSumWork / (float)g_partialCount_Alphasense;
	g_internal.aux = (float)g_partialSumAux / (float)g_partialCount_Alphasense;
	memcpy_ram2ram(ps,&g_internal,sizeof(ALPHASENSE_STATS));
	g_partialSumWork = 0;
	g_partialSumAux = 0;
	g_partialCount_Alphasense = 0;
}

void alphasense_init( void )
{
	
}

void alphasense_adc_getValue( void )
{
	//ALPHASENSE_STATS * const ps = &g_internal;
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	uint8_t inputgain = 1;//, elements = 20;
	//char szBUF[64];
	
	//debug_string(NORMAL,PSTR("Alphasense_adc_getValue\r\n"),true);

	
	// DISABLE jtag - it locks the upper 4 pins of PORT B
	CCP       = CCP_IOREG_gc;    // Secret handshake
	MCU.MCUCR = 0b00000001;
	
	PORTB.PIN0CTRL = PORT_OPC_TOTEM_gc; // Auxiliary Electrode
	PORTB.PIN2CTRL = PORT_OPC_TOTEM_gc; // Working Electrode
	PORTB.PIN6CTRL = PORT_OPC_TOTEM_gc; // GND x offset
	
	adc_read_configuration(&ALPHASENSE_ADC, &adc_conf);
	adcch_read_configuration(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH, &adcch_conf);
	
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adc_write_configuration(&ALPHASENSE_ADC, &adc_conf);
	
	
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN6, ADCCH_NEG_NONE, inputgain);
	adcch_write_configuration(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH, &adcch_conf);
	adc_enable(&ALPHASENSE_ADC);
	adc_start_conversion(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	adc_wait_for_interrupt_flag(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	adc_start_conversion(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	adc_wait_for_interrupt_flag(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	const int16_t off = adc_get_result(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	adc_disable(&ALPHASENSE_ADC);
	//sprintf_P(szBUF,PSTR("Offset: %u\t"),off);
	//debug_string(NORMAL,szBUF,false);
	
	
	//adcch_set_input(&adcch_conf, ADCCH_POS_BANDGAP, ADCCH_NEG_NONE, inputgain);
	//adcch_write_configuration(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH, &adcch_conf);
	//adc_enable(&ALPHASENSE_ADC);
	//adc_start_conversion(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	//adc_wait_for_interrupt_flag(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	//const uint16_t gain = adc_get_result(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	//adc_disable(&ALPHASENSE_ADC);
	////sprintf_P(szBUF,PSTR("gain: %u\t"),gain);
	////debug_string(NORMAL,szBUF,false);
	
	
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, inputgain);
	adcch_write_configuration(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH, &adcch_conf);
	adc_enable(&ALPHASENSE_ADC);
	adc_start_conversion(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	adc_wait_for_interrupt_flag(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	adc_start_conversion(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	adc_wait_for_interrupt_flag(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	const int16_t adc_w = adc_get_result(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH)-off;
	adc_disable(&ALPHASENSE_ADC);
	//sprintf_P(szBUF,PSTR("ADC_WORK: %u\t"),adc_w);
	//debug_string(NORMAL,szBUF,false);
	
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN2, ADCCH_NEG_NONE, inputgain);
	adcch_write_configuration(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH, &adcch_conf);
	adc_enable(&ALPHASENSE_ADC);
	adc_start_conversion(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	adc_wait_for_interrupt_flag(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	adc_start_conversion(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	adc_wait_for_interrupt_flag(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
	const int16_t adc_a = adc_get_result(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH)-off;
	adc_disable(&ALPHASENSE_ADC);
	//sprintf_P(szBUF,PSTR("ADC_AUX: %u\r\n"),adc_a);
	//debug_string(NORMAL,szBUF,false);	
	
	
	
	//adc_enable(&ALPHASENSE_ADC);
	//
	//for (int i=0;i<elements;i++)
	//{
		//adc_start_conversion(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
		//adc_wait_for_interrupt_flag(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
		//adc_start_conversion(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
		//adc_wait_for_interrupt_flag(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
		//result[i]=adc_get_result(&ALPHASENSE_ADC, ALPHASENSE_ADC_CH);
//
	//}
	
	//adc_disable(&ALPHASENSE_ADC);
	
	PORTB.PIN0CTRL = PORT_OPC_PULLUP_gc;
	PORTB.PIN2CTRL = PORT_OPC_PULLUP_gc;
	PORTB.PIN6CTRL = PORT_OPC_PULLUP_gc;
	
	g_recordingData_Alphasense = true;
	g_partialSumWork+=(int32_t)adc_w;
	g_partialSumAux+=(int32_t)adc_a;
	g_partialCount_Alphasense++;
	g_recordingData_Alphasense = false;
	
	//sprintf_P(szBUF,PSTR("Sample: %u\tPartialSumWork: %lu\tPartialSumAux: %lu\r\n"),g_partialCount, g_partialSumWork,g_partialSumAux);
	//debug_string(NORMAL,szBUF,false);
	
	//ps->working = adc_w;
	//ps->aux = adc_a;
}

