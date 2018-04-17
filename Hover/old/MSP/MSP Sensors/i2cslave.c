/*
 * i2cslave.c
 *
 *  Created on: Feb 22, 2018
 *      Author: mostly BHill
 */


#include  "msp430.h"
#define uart_max 12
#define i2c_max  12

unsigned char tx_data_str[uart_max], rx_data_str[uart_max], rx_flag=0, dec_str[6], eos_flag=0;
char dec_char[6];
int tx_ptr,e_tx_ptr;
unsigned char i2cTXData[i2c_max],i2cRXData[i2c_max];
volatile int i2cTXData_ptr=0,i2cRXData_ptr=0,i2crxflag=0;
volatile int i2cmode=0;


void i2c_slave_init(int address){
	BCSCTL1 = CALBC1_16MHZ;                    // Set DCO
	DCOCTL = CALDCO_16MHZ;

	P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
	P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0

	UCB0CTL1 |= UCSWRST;                      // Enable SW reset
	UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C Slave, synchronous mode
	UCB0I2COA = address;                         // Own Address is input
	UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
	UCB0I2CIE |= UCSTTIE;                     // Enable STT interrupt
	IE2 |= (UCB0TXIE+UCB0RXIE);                          // Enable TX interrupt
	i2cmode=0;

	UCA0CTL1 |= UCSSEL_2;                     // SMCLK
	UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
	__bis_SR_register(GIE);       			// interrupts enabled
}






#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
	if (IE2&UCA0TXIE){									//portion of uart_write_fast_string
		tx_ptr++;
		if (tx_ptr<e_tx_ptr)
			UCA0TXBUF=tx_data_str[tx_ptr];
		else{
			while (!(IFG2&UCA0TXIFG));
			UCA0TXBUF='\n';
			while (!(IFG2&UCA0TXIFG));
			UCA0TXBUF='\r';
			IE2 &=~ UCA0TXIE;
		}
	}
	else{

		if (i2cmode){
			UCB0TXBUF = i2cTXData[i2cTXData_ptr];                       // TX data on i2c slave bus
			i2cTXData_ptr++;
		}
		else{
			i2cRXData[i2cRXData_ptr]=UCB0RXBUF;							// rx data on i2c slave bus
			i2cRXData_ptr++;
			i2crxflag++;
		}

	}



}

//  Place data in RX-buffer and set flag
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	volatile char temp;
	if(IFG2 & UCA0RXIFG){							// Receive data on UART
		// TODO: UART Check?
	}
	if(IFG2 & UCB0TXIFG){							// detect beginning of i2c in slave-master mode
		i2cmode=1;
		if (UCB0STAT&UCSTTIFG){
			UCB0STAT &= ~(UCSTPIFG + UCSTTIFG);       // Clear interrupt flags
			i2cTXData_ptr=0;
		}	// Increme
	}
	if(IFG2&UCB0RXIFG){								// detect beginning of i2c in master-slave mode
		i2cmode=0;
		if (UCB0STAT&UCSTTIFG){
			UCB0STAT &= ~(UCSTPIFG + UCSTTIFG);       // Clear interrupt flags
			i2cRXData_ptr=0;
		}	// Increment data
	}

}


void conv_hex_dec(int val){
	volatile int temp,prev;
	unsigned int divider=10000;
	volatile int n=1,z=0,neg=0;
	dec_str[0]='0';
	if (val<0){
		neg=1;
		val=val*(-1);
		dec_str[0]='-';
	}
	prev=0;
	for(n=1;n<6;n++){
		temp=(val-prev)/divider;


		dec_str[n]=temp+0x30;
		prev=prev+(temp*divider);


		divider=divider/10;
	}

}

void unsigned_conv_hex_dec(int val){
	volatile unsigned int temp,prev;
	unsigned int divider=10000;
	volatile unsigned int n=1,z=0,neg=0;
	dec_str[0]='0';
	prev=0;
	for(n=1;n<6;n++){
		temp=(val-prev)/divider;


		dec_str[n]=temp+0x30;
		prev=prev+(temp*divider);


		divider=divider/10;
	}

}

int conv_dec_hex ( void ){
	volatile int num,k,temp;
	num=0;
	for (k=1;k<6;k++){
		num*=10;
		temp=dec_char[k];
		if (dec_char[k]>0x39)
			return 0x7FFF;
		if (dec_char[k]<0x30)
			return 0x7fff;
		num+=(dec_char[k]-0x30);
	}
	if (dec_char[0]=='-')
		num=num*(-1);
	return num;
}

