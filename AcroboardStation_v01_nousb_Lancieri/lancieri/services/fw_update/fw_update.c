/*
 * fw_update.c
 *
 * Created: 04/08/2014 12:13:03
 *  Author: fabio
 */ 

 
 #include <asf.h>
 #include <stdio.h>
 #include "progmem.h"
 #include <conf_board.h>
 #include <conf_usart_serial.h>


 #include "globals.h"

 #include "config/conf_usart_serial.h"
 #include "drivers/sim900/sim900.h"
 #include "drivers/usart_interrupt/cbuffer_usart.h"
 #include "devices/statusled/status_led.h"
 #include "devices/AT24CXX/AT24CXX.h"
 #include "../config/config.h"
 #include "../libemqtt/libemqtt.h"

 #include "fw_update.h"
 
 static uint32_t g_timeBegin = 0;
 
 static void fw_update_timer(uint32_t timeNow)
 {
	rtc_set_alarm_relative(5);
	gpio_toggle_pin(STATUS_LED_PIN);

	wdt_reset();
	 
	if(g_timeBegin==0) g_timeBegin = timeNow;
	if( (timeNow-g_timeBegin) > 3600 ) {
		wdt_reset_mcu();
	}
	 
 }
 
 uint16_t fw_update_init( const char * pPara )
{

	DEBUG_PRINT_FUNCTION_NAME(VERBOSE);

	if(0!=sim900_GPRS_check_line())
	{
		uint8_t ii=0;
		while(1) {
			if(++ii==10) {
				wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_8CLK);
				wdt_enable();
				wdt_reset_mcu();
				delay_ms(1000);
			}
			if(0==sim900_init()) {
				break;
			}
		}
	}

	g_timeBegin = 0;
	rtc_set_callback(fw_update_timer);
	rtc_set_alarm_relative(5);
	wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_8KCLK);
	wdt_enable();


	return 0;
 }

#include "hex_processor.h"

uint16_t fw_update_process_hex_rec(const HEX_READER_RECORD* const pRec )
{


}


uint16_t fw_update_process_hex_chunk( const char * const pBuf,uint16_t size, uint16_t * const numLines )
{
	if( !(size>0) ) return 1;
	if(pBuf[0]!=':') return 2;
	
	uint16_t err = hex_processor_init(pBuf,size);
	
	HEX_READER_RECORD aRec;
	char status = 0;
	uint16_t nl = 0;

	while(err==0) {
		err = hex_processor_get_rec(&aRec,&status);
		
		if (status==EOF)
		{
			return 2;
		}
		
		hex_processor_verify_rec(&aRec);
		
		fw_update_process_hex_rec(&aRec);
		
		nl++;
	}
	
	*numLines = nl;
	return 0;
}

#include "eeprom_manager.h"


 
 void fw_update_run( const char * pPara )
{
	 
	DEBUG_PRINT_FUNCTION_NAME(VERBOSE);

	char szGetQuery[256];
	static const uint16_t SIZE_GET_QUERY = sizeof(szGetQuery);

	char szGetAnswer[1024];
	static const uint16_t SIZE_GET_ANSWER = sizeof(szGetAnswer);


	cfg_get_service_url_send(szGetQuery,64);
	uint16_t l1 = strnlen(szGetQuery,64)-1;

	snprintf_P(szGetQuery+l1,SIZE_GET_QUERY-l1,PSTR("/FW_GET?AWSID="));
	l1 += strnlen(szGetQuery+l1,64);

	cfg_get_aws_id(szGetQuery+l1,64);
	l1 += strnlen(szGetQuery+l1,64);

	snprintf_P(szGetQuery+l1,SIZE_GET_QUERY-l1,PSTR("&BSIZE=%d&LID="),SIZE_GET_ANSWER);
	l1 += strnlen(szGetQuery+l1,64);

	uint16_t err = sim900_GPRS_simple_open();

	em_init();

	uint16_t lid=0;
	while(1) {
		uint16_t lenRetBuf=SIZE_GET_ANSWER;

		snprintf_P(szGetQuery+l1,SIZE_GET_QUERY-l1,PSTR("%d"),lid);
		err = sim900_http_get(szGetQuery,RAM_STRING,szGetAnswer,&lenRetBuf);
		sim900_http_close();

		if(err!=0) {
			continue;
		}


		uint16_t numLines;

		err = fw_update_process_hex_chunk(szGetAnswer,lenRetBuf,&numLines);

		if (err==2)	{
			//We got EOF while processing the chunk
			
		} else 	if(err!=0) {
			continue;
		}
		
		lid += numLines;
	}

	 
quit_update:

	sim900_http_close();
	sim900_GPRS_close();

 }


 void fw_update_run_test( const char * pPara )
 {
	 
	DEBUG_PRINT_FUNCTION_NAME(VERBOSE);

	char szGetQuery[256];
	static const uint16_t SIZE_GET_QUERY = sizeof(szGetQuery);

	char szGetAnswer[1024];
	static const uint16_t SIZE_GET_ANSWER = sizeof(szGetAnswer);

	uint16_t err = sim900_GPRS_simple_open();

	for (uint8_t b=0;b<8;++b)
	{
		uint16_t lenRetBuf=SIZE_GET_ANSWER;

		sprintf_P(szGetQuery,PSTR("http://www.acrotec.it/cimameteo/blob/blob%d"),b);
		uint16_t l1 = strnlen(szGetQuery,64)-1;



		err = sim900_http_get(szGetQuery,false,szGetAnswer,&lenRetBuf);
		sim900_http_close();

		if(err!=0) {
			return err;
		}
	}

	 
 }

