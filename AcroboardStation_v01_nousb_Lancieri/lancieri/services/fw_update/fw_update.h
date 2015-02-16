/*
 * fw_update.h
 *
 * Created: 04/08/2014 12:13:19
 *  Author: fabio
 */ 


#ifndef FW_UPDATE_H_
#define FW_UPDATE_H_


 uint16_t fw_update_init(const char * pPara);
 void fw_update_run(const char * pPara);
void fw_update_run_test( const char * pPara );


#endif /* FW_UPDATE_H_ */