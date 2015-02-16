/*
 * hex_processor.c
 *
 * Created: 01/10/2014 16:16:09
 *  Author: fabio
 */ 


#include <asf.h>
#include <stdio.h>
#include "progmem.h"
#include <conf_board.h>
#include <conf_usart_serial.h>


#include "globals.h"

#include "../config/config.h"

#include "config/conf_usart_serial.h"

#include "drivers/sim900/sim900.h"
#include "drivers/usart_interrupt/cbuffer_usart.h"

#include "devices/raingauge/pulse_raingauge.h"
#include "devices/statusled/status_led.h"
#include "devices/AT24CXX/AT24CXX.h"
#include "devices/Voltmeter/voltmeter.h"

#include "../libemqtt/libemqtt.h"

#include "services/datalogger/datalogger.h"
#include "services/fw_update/fw_update.h"
#include "services/fw_update/hex_processor.h"


static uint16_t g_eeBuf_idx = 0;
static uint16_t g_eeBuf_size = 0;
static char * g_eeBUF = NULL;


static uint8_t ascii_hex(const uint8_t c);

uint16_t hex_processor_verify_chunk_crc(void * pBuf,uint16_t len)
{
	uint32_t c1 = crc_io_checksum(pBuf,len,CRC_32BIT);
}

uint16_t hex_processor_init(char * const pBuf,uint16_t size)
{
	g_eeBuf_idx = 0;
	g_eeBuf_size = size-8;
	g_eeBUF = pBuf+8;
	
	const uint8_t v8 = ascii_hex(pBuf[0]);
	const uint8_t v7 = ascii_hex(pBuf[1]);
	const uint8_t v6 = ascii_hex(pBuf[2]);
	const uint8_t v5 = ascii_hex(pBuf[3]);
	const uint8_t v4 = ascii_hex(pBuf[4]);
	const uint8_t v3 = ascii_hex(pBuf[5]);
	const uint8_t v2 = ascii_hex(pBuf[6]);
	const uint8_t v1 = ascii_hex(pBuf[7]);
	
	const uint16_t c1 = ( (uint16_t) ((v3<<4) | v4 )<<8 )  | (v1<<4) | v2;
	const uint16_t c2 = ( (uint16_t) ((v7<<4) | v8 )<<8 )  | (v5<<4) | v6;
	
	const uint32_t cc = ((uint32_t) c1) << 16 | c2;
	
	return hex_processor_verify_chunk_crc(g_eeBUF,g_eeBuf_size);
}



//static uint8_t hex_reader_feed_buf( void )
//{
	////if(g_hex_iter.pg>7) {return -1;}
	////AT24CXX_ReadBlock(g_hex_iter.pg,g_hex_iter.msb,0,g_eeBUF  ,256);
	////g_hex_iter.msb++;
	////if (g_hex_iter.msb==0)
	////{
		////g_hex_iter.pg++;
	////}
	//return 0;
//}

static uint8_t hex_reader_get_char(char * const status)
{
	if (g_eeBuf_idx==g_eeBuf_size)
	{
		*status = EOF;
	}

	return g_eeBUF[g_eeBuf_idx++];
}

static uint8_t hex_reader_trim_to_record(char * const status)
{
	char ret = hex_reader_get_char(&status);
	if(status==EOF) return -1;
	if (ret==':') return 0;
	
	ret = hex_reader_get_char(&status);
	if(status==EOF) return -1;
	if (ret==':') return 0;
	
	ret = hex_reader_get_char(&status);
	if(status==EOF) return -1;
	if (ret==':') return 0;
	
	return 1;
}

uint8_t ascii_hex(const uint8_t c)
{
	if((c>47) && (c<58)) { // 0 to 9
		return (c-48);
		} else if((c>64) && (c<71)) { // A to F
		return (c-65);
		} else if((c>96) && (c<103)) { // a to f
		return (c-97);
	}
	
	//ERROR TO HANDLE
	
	return 0xFF;
}

static uint8_t hex_reader_decode_byte(uint8_t * val, char * const status)
{
	const uint8_t l1 = ascii_hex(hex_reader_get_char(&status));
	if(status==EOF) return -1;

	const uint8_t l2 = ascii_hex(hex_reader_get_char(&status));
	if(status==EOF) return -1;

	*val = (l1<<4) | l2;

	return 0;
}

static uint8_t hex_reader_decode_len(uint8_t * len, char * const status)
{
	return hex_reader_decode_byte(len, status);
}

static uint8_t hex_reader_decode_rectype(uint8_t * type, char * const status)
{
	return hex_reader_decode_byte(type, status);
}

static uint8_t hex_reader_decode_checksum(uint8_t * val, const char * status)
{
	return hex_reader_decode_byte(val, status);
}

static uint8_t hex_reader_decode_address(uint16_t * val, char * const status)
{
	const uint8_t v1 = ascii_hex(hex_reader_get_char(&status));
	if(status==EOF) return -1;
	const uint8_t v2 = ascii_hex(hex_reader_get_char(&status));
	if(status==EOF) return -1;
	const uint8_t v3 = ascii_hex(hex_reader_get_char(&status));
	if(status==EOF) return -1;
	const uint8_t v4 = ascii_hex(hex_reader_get_char(&status));
	if(status==EOF) return -1;

	*val = ( (uint16_t) ((v3<<4) | v4 )<<8 )  | (v1<<4) | v2;
	
	return 0;
}


uint8_t hex_processor_get_rec( HEX_READER_RECORD * pRec, char * const status )
{
	
	if( (hex_reader_trim_to_record(status)) || (status==EOF))
	{
		//Start of record missing
		return 1;
	}

	uint8_t len;
	hex_reader_decode_len(&len, status);
	if(status==EOF) return -1;

	pRec->len = len;
	
	uint16_t address;
	hex_reader_decode_address(&address, status);
	if(status==EOF) return -1;

	pRec->address = address;
	
	uint8_t type;
	hex_reader_decode_rectype(&type, status);
	if(status==EOF) return -1;

	pRec->type = type;

	uint8_t * const pData = pRec->data;
	for(uint8_t idx=0;idx<len;++idx) {
		hex_reader_decode_byte(pData+idx, status);
		if(status==EOF) return -1;
	}

	uint8_t chk;
	hex_reader_decode_checksum(&chk, status);
	if(status==EOF) return -1;

	pRec->chk = chk;
	
	return 0;
	
}

uint8_t hex_processor_verify_rec(const HEX_READER_RECORD * const pRec )
{
	uint8_t i=pRec->len;
	int8_t v = i;

	while(i--)
	{
		const int8_t t = pRec->data[i];
		v += t;
	}
	
	int8_t t = pRec->type;
	v+=t;
	
	t = (int8_t)(pRec->address & 0xFF);
	v += t;
	
	t = (int8_t)((pRec->address>>8)  & 0xFF);
	v += t;
	
	if ((v - pRec->chk) == 0)
	{
		return 0;
	}
	
	
	return 1;
}
