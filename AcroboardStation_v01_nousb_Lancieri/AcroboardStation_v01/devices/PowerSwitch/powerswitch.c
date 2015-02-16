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
#include "conf_board.h"
#include "globals.h"
#include "powerswitch.h"

void powerSwitch_init(void){
	ioport_configure_pin(MCU_SWITCH_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
}

void powerSwitch_hardReset(void){
	ioport_configure_pin(MCU_SWITCH_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	delay_ms(100);
	ioport_configure_pin(MCU_SWITCH_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
}