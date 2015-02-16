/*
 * eeprom_manager.c
 *
 * Created: 02/10/2014 14:04:54
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

#include "eeprom_manager.h"

typedef struct HEX_READER_EEPROM_ITERATOR
{
	uint8_t i;
	uint8_t pg;
	uint8_t msb;
	uint8_t rem;
} HEX_READER_EEPROM_ITERATOR;

static HEX_READER_EEPROM_ITERATOR g_hex_iter;

uint16_t em_init(void) {
	
	g_hex_iter.pg = 0;
	g_hex_iter.msb = 1;
	g_hex_iter.rem = 0;

	
	return 0;
	
}

uint16_t em_insert(const HEX_READER_RECORD* const pRec)
{
	
}