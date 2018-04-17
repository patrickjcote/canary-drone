/*
 * i2slave.h
 *
 *  Created on: Feb 22, 2018
 *      Author: pcote
 */

#ifndef I2CSLAVE_H_
#define I2CSLAVE_H_

extern unsigned char tx_data_str[24], rx_data_str[24],rx_flag ,dec_str[7],eos_flag;
extern char dec_char[6];
void conv_hex_dec(int);
void unsigned_conv_hex_dec(int);
int conv_dec_hex (void);
void  i2c_slave_init(int);
//void uart_write_fast_string(int, int);
extern unsigned char i2cTXData[64],i2cRXData[64];
extern volatile int i2cTXData_ptr,i2cRXData_ptr,i2crxflag;
extern volatile int i2cmode;


#endif /* I2CSLAVE_H_ */
