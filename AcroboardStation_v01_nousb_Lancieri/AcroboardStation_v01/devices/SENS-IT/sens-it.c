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
#include "board.h"
#include "twi_master.h"
#include <stdio.h>

#include "conf_board.h"
#include "globals.h"
#include <conf_usart_serial.h>
#include "config/conf_usart_serial.h"

#include "devices/SENS-IT/sens-it.h"

char g_szBuf[64], g_target_gas_c [5];
uint8_t g_idx = 0;
int16_t g_temp = 0;
uint16_t g_readigNumber[3], g_err[3];
uint32_t g_gas[3];



void sens_it_init(void)
{
	static usart_rs232_options_t RS485_SERIAL_OPTIONS = {
		.baudrate = USART_SENSIT_BAUDRATE,
		.charlength = USART_CHAR_LENGTH,
		.paritytype = USART_PARITY,
		.stopbits = USART_STOP_BIT
	};
	sysclk_enable_module(SYSCLK_PORT_C,PR_USART0_bm);
	usart_serial_init(USART_SENSIT, &RS485_SERIAL_OPTIONS);
	usart_set_rx_interrupt_level(USART_SENSIT,USART_INT_LVL_LO); // Occhio alla priorità
	usart_rx_enable(USART_SENSIT);
	usart_tx_enable(USART_SENSIT);
	
	//sysclk_enable_module(SYSCLK_PORT_C,PR_USART1_bm);
	//usart_serial_init(USART_SENSIT_2, &RS485_SERIAL_OPTIONS);
	//usart_set_rx_interrupt_level(USART_SENSIT_2,USART_INT_LVL_LO); // Occhio alla priorità
	//usart_rx_enable(USART_SENSIT_2);
	//usart_tx_enable(USART_SENSIT_2);
}
void sens_it_resetStats(void)
{
	//memset(g_gas,0,3);
	//memset(g_err,0,3);
	//memset(g_readigNumber,0,3);
	for (int i = 0; i <= 2 ; i++)
	{
		g_gas[i] = 0;
		g_err[i] = 0;
		g_readigNumber[i] = 0;
	}
	//
	memset(g_target_gas_c,0,5);
	memset(g_szBuf,0,64);
}

void sens_it_getStats(SENSIT_STATS * const ps)
{
	char szBuf[64];
	for (int i = 0; i <= 2 ; i++)
	{
		if(g_readigNumber[i])
		{
			ps->gas[i] = (g_gas[i] / g_readigNumber[i]);
			ps->error_check[i] = (g_err[i] / g_readigNumber[i]);
			ps->readingNumber[i] = g_readigNumber[i];
			//ps->temp = g_temp;
		}
		else
		{
			ps->gas[i] = -9998;
			ps->error_check[i] = -9998;
			ps->readingNumber[i] = 0;
			//ps->temp = g_temp;
		}
		sprintf_P(szBuf,PSTR("\r\n%d - gas = %ld\terr = %d\tnum = %u"),
		i,
		(g_gas[i] / g_readigNumber[i]),
		(g_err[i] / g_readigNumber[i]),
		g_readigNumber[i]
		);
		debug_string(NORMAL,szBuf,RAM_STRING);
	}
	//memcpy_ram2ram(ps->target_gas,g_target_gas_c,5*sizeof(char));
	
	sens_it_resetStats();
}

void sens_it_triggerReading(uint8_t address)
{
	char szBuf[128], sensorReading[64];
	uint32_t gas = 0, res = 0;
	int16_t temp = 0, fan = 0, sign=0;
	uint8_t i=0, field = 0, error_check = 0, target_gas_c [5], idx = 0, gas_flag, target_gas;
	memset(g_szBuf,0,64);
	debug_string(NORMAL,PSTR("Sending reading command\r\n"),true);
	//usart_tx_enable(USART_SENSIT);
	usart_rx_disable(USART_SENSIT);
	gpio_set_pin_high(SENSIT_USART_TX_ENABLE);
	delay_ms(10);
	usart_putchar(USART_SENSIT,'&');
	usart_putchar(USART_DEBUG,'&');
	delay_ms(5);
	usart_putchar(USART_SENSIT,address);
	usart_putchar(USART_DEBUG,address);
	usart_rx_enable(USART_SENSIT);
	//usart_tx_disable(USART_SENSIT);
	
	delay_us(600);
	
	gpio_set_pin_low(SENSIT_USART_TX_ENABLE);
	delay_ms(125);
	
	memcpy_ram2ram(sensorReading,g_szBuf,63*sizeof(char));
	
	gas_flag = 1;
	char c;
	while(gas_flag)
	{
		
		const char c=sensorReading[i++];
		//usart_putchar(USART_DEBUG,c);
		if (c == ';')
		{
			field++;
		}
		else{
			if (c == '.')
				continue;
			if(field==2) //target gas
			{
				target_gas_c[idx++] = c;
			}
			if(field==3) //gas concentration * 100
			{
				gas=gas*10+c-48;
			}
			else if(field==4) //sensing resistance * 10
			{
				res=res*10+c-48;
			}
			else if(field==5) //temperature * 10
			{
				if(c=='+')
					sign=1;
				else if(c=='-')
					sign=-1;
				else
					temp=temp*10+c-48;
			}
			else if(field==6)
			{
				fan=fan*10+c-48;
			}
			else if(field==7)
			{
				error_check=error_check*10+c-48;
			}
		}
		if(c=='\r' || c=='\n' || i >= 48)
		{
			temp *=sign;
			g_idx = 0;
			gas_flag =0;
			target_gas = target_gas_c[1] - 48;
			g_readigNumber[target_gas-1]++;
		}
		
	}
	
	
	switch(target_gas)
	{
		case(1):
		memcpy_ram2ram(target_gas_c, "CO", 3*sizeof(char));
		break;
		case(2):
		memcpy_ram2ram(target_gas_c, "NO2", 4*sizeof(char));
		break;
		case(3):
		memcpy_ram2ram(target_gas_c, "O3", 3*sizeof(char));
		break;
		case(4):
		memcpy_ram2ram(target_gas_c, "CH4", 4*sizeof(char));
		break;
		case(5):
		memcpy_ram2ram(target_gas_c, "NOx", 4*sizeof(char));
		break;
		case(6):
		memcpy_ram2ram(target_gas_c, "C6H6", 5*sizeof(char));
		break;
	};
	
	sprintf_P(szBuf,PSTR("\r\nfield = %u\ttarget_gas = %s\tgas = %lu\tres = %lu\ttemp = %u\tfan = %u\terr = %u"),
	field,
	target_gas_c,
	gas,
	res,
	temp,
	fan,
	error_check
	);
	debug_string(VERBOSE,szBuf,false);
	
	memcpy_ram2ram(g_target_gas_c,target_gas_c,5*sizeof(char));
	g_gas[target_gas-1] += gas;
	g_err[target_gas-1] +=error_check;
	//g_temp = temp;
	
	sprintf_P(szBuf,PSTR("\r\ngas_part_sum = %lu\terr_part_sum = %u\tReadNum = %u"),
	g_gas[target_gas-1],
	g_err[target_gas-1],
	g_readigNumber[target_gas-1]
	);
	debug_string(VERBOSE,szBuf,false);
	
}

static bool usartc_USARTC0_RX_CBuffer_Complete(void)
{
	USART_t * const ad = USART_SENSIT;
	const uint8_t dt = ad->DATA;
	usart_putchar(USART_DEBUG,dt);
	//if(dt=='#')
	//{
		//idx=0;
		////memcpy_ram2ram(g_SensorString,g_szBuf,sizeof(g_szBuf[0]));
		//g_flag = 1;
		//
	//}
	g_szBuf[g_idx++]=dt;
}

ISR(USARTC0_RXC_vect)
{
	usartc_USARTC0_RX_CBuffer_Complete();
}