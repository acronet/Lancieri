/*
 * eeprom_manager.h
 *
 * Created: 02/10/2014 14:05:08
 *  Author: fabio
 */ 


#ifndef EEPROM_MANAGER_H_
#define EEPROM_MANAGER_H_



uint16_t em_init(void);
uint16_t em_insert(const HEX_READER_RECORD* const pRec);


#endif /* EEPROM_MANAGER_H_ */
