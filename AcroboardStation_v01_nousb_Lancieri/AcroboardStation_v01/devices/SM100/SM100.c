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

#include <stddef.h>
#include <avr/pgmspace.h>

#include "asf.h"
#include <stdio.h>
#include "progmem.h"
#include "devices/statusled/status_led.h"
#include "devices/USBVbusMonitor/VBusMon.h"

#include "globals.h"
#include "devices/SM100/SM100.h"

volatile bool g_recordingData_SM100 = false;
SM100_STATS g_internal_SM100;
int32_t g_partialSumA = 0, g_partialSumB = 0;
int16_t g_partialCount_SM100 = 0 , g_lastA = 0, g_lastB = 0;

static uint8_t ReadCalibrationByte( uint8_t index )
{
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return( result );
}

void SM100_get_stats( SM100_STATS * const ps )
{
	while(g_recordingData_SM100);
	//meglio assegnare direttamente i due parametri?
	g_internal_SM100.lastA = g_lastA;
	g_internal_SM100.lastB = g_lastB;
	g_internal_SM100.sensorA = (float)g_partialSumA / (float)g_partialCount_SM100;
	g_internal_SM100.sensorB = (float)g_partialSumB / (float)g_partialCount_SM100;
	g_internal_SM100.ratioA = (float)(g_internal_SM100.sensorA*10) / 4095.0F * 74.42455243F; // 0.7442455243F --> Voltage divider Factor
	g_internal_SM100.ratioB = (float)(g_internal_SM100.sensorB*10) / 4095.0F * 74.42455243F; // 0.7442455243F --> Voltage divider Factor
	g_internal_SM100.lastA = (float)(g_lastA*10) / 4095.0F * 74.42455243F; // 0.7442455243F --> Voltage divider Factor
	g_internal_SM100.lastB = (float)(g_lastB*10) / 4095.0F * 74.42455243F; // 0.7442455243F --> Voltage divider Factor
	g_internal_SM100.counter = g_partialCount_SM100;
	memcpy_ram2ram(ps,&g_internal_SM100,sizeof(SM100_STATS));
	g_partialSumA = 0;
	g_partialSumB = 0;
	g_partialCount_SM100 = 0;
}

void SM100_init(void)
{
	//Configure the soil moisture switch
	ioport_configure_pin(EXTV33_SWITCH, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(EXTV33_SWITCH , LOW);
	
	PORTB.DIR &= 0xBA;
	ADCB.CALL = ReadCalibrationByte( ADCBCAL0 );
	ADCB.CALH = ReadCalibrationByte( ADCBCAL1 );
	
	g_partialSumA = 0;
	g_partialSumB = 0;
	g_partialCount_SM100 = 0;
}

static void SM100_start(void)
{
	ioport_set_pin_level(EXTV33_SWITCH , HIGH);
	delay_s(1);
}

void SM100_getValue(void)
{	
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	
	PORTB.PIN0CTRL= PORT_OPC_TOTEM_gc; //Voltage Reference
	PORTB.PIN2CTRL= PORT_OPC_TOTEM_gc;
	PORTB.PIN4CTRL= PORT_OPC_TOTEM_gc;
	PORTB.PIN6CTRL= PORT_OPC_TOTEM_gc;
	
	adc_read_configuration(&SM100_ADC, &adc_conf);
	adcch_read_configuration(&SM100_ADC, SM100_ADC_CH, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, SM100_ADC_REF);//
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 100000UL);
	
	adc_write_configuration(&SM100_ADC, &adc_conf);
	
	SM100_start();
	
	delay_ms(100);
		
	adcch_set_input(&adcch_conf, SM100_ADC_GND, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&SM100_ADC, SM100_ADC_CH, &adcch_conf);
	delay_ms(2);
	adc_enable(&SM100_ADC);
	adc_start_conversion(&SM100_ADC, SM100_ADC_CH);
	adc_wait_for_interrupt_flag(&SM100_ADC, SM100_ADC_CH);
	adc_start_conversion(&SM100_ADC, SM100_ADC_CH);
	adc_wait_for_interrupt_flag(&SM100_ADC, SM100_ADC_CH);
	const int16_t offset = adc_get_result(&SM100_ADC, SM100_ADC_CH);
	adc_disable(&SM100_ADC);
	
	adcch_set_input(&adcch_conf, SM100_ADC_SENSOR_B, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&SM100_ADC, SM100_ADC_CH, &adcch_conf);
	delay_ms(2);
	adc_enable(&SM100_ADC);
	adc_start_conversion(&SM100_ADC, SM100_ADC_CH);
	adc_wait_for_interrupt_flag(&SM100_ADC, SM100_ADC_CH);
	adc_start_conversion(&SM100_ADC, SM100_ADC_CH);
	adc_wait_for_interrupt_flag(&SM100_ADC, SM100_ADC_CH);
	const int16_t resultB = adc_get_result(&SM100_ADC, SM100_ADC_CH)-offset;
	adc_disable(&SM100_ADC);
	
	adcch_set_input(&adcch_conf, SM100_ADC_SENSOR_A, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&SM100_ADC, SM100_ADC_CH, &adcch_conf);
	delay_ms(2);
	adc_enable(&SM100_ADC);
	adc_start_conversion(&SM100_ADC, SM100_ADC_CH);
	adc_wait_for_interrupt_flag(&SM100_ADC, SM100_ADC_CH);
	adc_start_conversion(&SM100_ADC, SM100_ADC_CH);
	adc_wait_for_interrupt_flag(&SM100_ADC, SM100_ADC_CH);
	const int16_t resultA = adc_get_result(&SM100_ADC, SM100_ADC_CH)-offset;
	adc_disable(&SM100_ADC);
	ioport_set_pin_level(EXTV33_SWITCH , LOW);
	
	PORTB.PIN0CTRL= PORT_OPC_PULLUP_gc; //Voltage Reference
	PORTB.PIN2CTRL= PORT_OPC_PULLUP_gc;
	PORTB.PIN4CTRL= PORT_OPC_PULLUP_gc;
	PORTB.PIN6CTRL= PORT_OPC_PULLUP_gc;
	
	g_recordingData_SM100 = true;
	g_lastA = resultA;
	g_lastB = resultB;
	g_partialSumA+=(int32_t)resultA;
	g_partialSumB+=(int32_t)resultB;
	g_partialCount_SM100++;
	g_recordingData_SM100 = false;
	
	
	//char szBUF[64];
	//sprintf_P(szBUF,PSTR("%u\t%u\t%u\t"),resultA,resultB,offset);
	//debug_string(NORMAL,szBUF,false);
	//sprintf_P(szBUF,PSTR("%lu\t%lu\t%u\r\n"),g_partialSumA,g_partialSumB,g_partialCount_SM100);
	//debug_string(NORMAL,szBUF,false);
}