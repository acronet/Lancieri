/*
 * hex_processor.h
 *
 * Created: 01/10/2014 16:16:38
 *  Author: fabio
 */ 


#ifndef HEX_PROCESSOR_H_
#define HEX_PROCESSOR_H_


typedef struct HEX_READER_RECORD
{
	uint8_t len;
	uint16_t address;
	uint8_t type;
	uint8_t data[255];
	int8_t  chk;
} HEX_READER_RECORD;

uint16_t hex_processor_init(char * const pBuf,uint16_t size);
uint8_t hex_processor_verify_rec(const HEX_READER_RECORD * const pRec );
uint8_t hex_processor_get_rec( HEX_READER_RECORD * pRec, char * const status );


#endif /* HEX_PROCESSOR_H_ */