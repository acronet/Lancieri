/**
 * \file
 *
 * \brief User board configuration template
 *
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

#define BOARD_XOSC_HZ                   32768
#define BOARD_XOSC_TYPE                 XOSC_TYPE_32KHZ
#define BOARD_XOSC_STARTUP_US           1000000


#define STATUS_LED_PIN		IOPORT_CREATE_PIN(PORTD,0)
#define RAINGAUGE1_SWITCH	IOPORT_CREATE_PIN(PORTR,0)
#define RAINGAUGE2_SWITCH	IOPORT_CREATE_PIN(PORTR,1)

#define GPRS_SWITCH			IOPORT_CREATE_PIN(PORTF, 3)
//#define GPRS_RESET			IOPORT_CREATE_PIN(PORTF, 1)
#define GPRS_UART_RX		IOPORT_CREATE_PIN(PORTE, 2)
#define GPRS_UART_TX		IOPORT_CREATE_PIN(PORTE, 3)
#define GPRS_STATUS			IOPORT_CREATE_PIN(PORTF, 1)

//#define EXEEPROM_SWITCH		IOPORT_CREATE_PIN(PORTD, 3)
#define EXEEPROM_SCL		IOPORT_CREATE_PIN(PORTE, 1)
#define EXEEPROM_SDA		IOPORT_CREATE_PIN(PORTE, 0)

#define USB_PROBE_PIN   IOPORT_CREATE_PIN(PORTD, 5)

//#define CONF_BOARD_USB_PORT

#define EEPROM_CHIP_ADDR       0x50        //!< TWI slave memory address
#define EEPROM_TWI_SPEED       250000       //!< TWI data transfer rate
#define EEPROM_TWI_PORT        (&TWIE)
#define TWI_MASTER_ADDRESS     0x99

#define AUX_TWI_PORT        (&TWIC)

//////////////////////////////////////////////////////////////////
// SENSORCODE: Define the board version
//#define BOARD_ACROSTATION_R08
#define BOARD_ACROSTATION_R09
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// SENSORCODE: Define here sensors' pins
#define BATTERY_VOLTMETER				ADCA
#define BATTERY_VOLTMETER_CH			ADC_CH0
#define BATTERY_VOLTMETER_PIN			ADCCH_POS_PIN1
#define BATTERY_VOLTMETER_SWITCH		IOPORT_CREATE_PIN(PORTF, 4)

#define LEVELGAUGE_SWITCH				IOPORT_CREATE_PIN(PORTA, 3)
#define EXTV33_SWITCH					IOPORT_CREATE_PIN(PORTA, 4)

#define MCU_SWITCH_PIN					IOPORT_CREATE_PIN(PORTD, 1)

#define VP61_PIN				IOPORT_CREATE_PIN(PORTA, 3)
#define VP61_GND				IOPORT_CREATE_PIN(PORTA, 5)
// SENSIT UART CONFIGURATION
#define SENSIT_USART_TX					IOPORT_CREATE_PIN(PORTC, 3)
#define SENSIT_USART_RX					IOPORT_CREATE_PIN(PORTC, 2)
#define SENSIT_USART_TX_ENABLE			IOPORT_CREATE_PIN(PORTC, 4)
#define SENSIT_USART_2_TX				IOPORT_CREATE_PIN(PORTC, 7)
#define SENSIT_USART_2_RX				IOPORT_CREATE_PIN(PORTC, 6)
#define SENSIT_USART_2_TX_ENABLE		IOPORT_CREATE_PIN(PORTC, 5)
//////////////////////////////////////////////////////////////////

#endif // CONF_BOARD_H
