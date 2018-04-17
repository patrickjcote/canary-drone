/*
 * main.c
 *
 *  Created on: March 9, 2017
 *      Author: Patrick Cote
 *
 */

// Includes
#include <msp430.h>
#include "i2cslave.h"
#include <msp430g2553.h>                // include necessary templates
#include <intrinsics.h>
#include <stdint.h>



#define BOARD_ADDRESS 0x08
#define DELAY 9600				// 9600cycles/16MHz = 600us
#define DELAY_MULTIPLIER 10 	// 10*600us = 6ms
#define TIMEOUT 10000				// ~9ms*100 = 0.9s
#define DEBUG 1
void sendFrame();

unsigned int canary[5] = {0,0,0,0,0};
volatile unsigned char command[22];

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;			// Stop watchdog timer

	volatile unsigned int k;
	unsigned int timeOut = 0;

	__delay_cycles(50000);
	BCSCTL1 = CALBC1_16MHZ;				// Set Clock Speed to 16MHz
	DCOCTL = CALDCO_16MHZ;
	for (k=0;k<200;k++)
		__delay_cycles(50000);


	// Init UART
	DCOCTL = 0;                             // Select lowest DCOx and MODx settings
	BCSCTL1 = CALBC1_16MHZ;                  // Set DCO
	DCOCTL = CALDCO_16MHZ;
	P1SEL = BIT2 ;                   		// P1.1 = RXD, P1.2=TXD
	P1SEL2 = BIT2 ;                  		// P1.1 = RXD, P1.2=TXD
	UCA0CTL0 |= UCSPB + UCPEN + UCPAR;		// 2 Stop Bits, Enable Parity, Even Parity
	UCA0CTL1 |= UCSSEL_2;                   // SMCLK
	UCA0BR0 = 160;                          // 16MHz 100k
	UCA0BR1 = 0;                            // 16MHz
	UCA0MCTL = UCBRS0;                      // Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**
	IE2 |= UCA0RXIE;                        // Enable USCI_A0 RX interrupt

	__delay_cycles(50000);
		i2c_slave_init(BOARD_ADDRESS); 		// Set slave address
		for (k=0;k<200;k++)
			__delay_cycles(50000);
		i2cTXData[0]=1;						// Dummy values for i2cTXData
		i2cTXData[1]=2;
		i2cTXData[2]=3;



	for (k=0;k<200;k++)
		__delay_cycles(50000);

	__bis_SR_register(GIE);       			// interrupts enabled

	canary[0]=900; canary[1]=1500; canary[2]=1500; canary[3]=1500; canary[4]=900;

	while(1){

		if (i2crxflag){					// if data received as the I2C slave
			__delay_cycles(5000);
			unsigned int i2cIn[6] = {0,0,0,0,0,0};
			for(k=1;k<7;k++)			// Shift by 1 because no register is sent?
				i2cIn[k] = i2cRXData[k*2-1] + (i2cRXData[k*2]<<8);

			canary[0] = i2cIn[1];
			canary[1] = i2cIn[2];
			canary[2] = i2cIn[3];
			canary[3] = i2cIn[4];
			canary[4] = i2cIn[5];
			i2crxflag=0;
			timeOut = 0;
		}

		//
		if(timeOut>TIMEOUT){
			canary[0]=900; canary[1]=1500; canary[2]=1500; canary[3]=1500; canary[4]=900;
			timeOut = TIMEOUT;
		}



        // Send a frame TODO: Put in Timer Interrupt
        sendFrame();

        for(k=0; k < DELAY_MULTIPLIER; k++)
        	__delay_cycles(DELAY);

        timeOut++;

	}//forever while loop
}// main()

void sendFrame()
{   // Convert from 1000-2000 to SBUS range (192, 1792)
	volatile unsigned int i, cmdByte, cmdBit;
	unsigned int sbus[5] = {0,0,0,0,0};

	for(i = 0; i < 5; i++)
	{
		sbus[i] = (canary[i] * 8) / 5  -  1408;
	}

	volatile unsigned int chNdx = 0;
	volatile unsigned int chBit = 0;
	volatile unsigned char currentBit;

	for(cmdByte = 0; cmdByte < 7; cmdByte++)
    {
		for(cmdBit = 0; cmdBit < 8; cmdBit++)
        {
			// Shift channel to isolate current bit
			currentBit = (sbus[chNdx] >> chBit) & 1;
			// Set cmdBit in the cmdByte to currentBit
			if(currentBit)
            {
				command[cmdByte] |= (1<<cmdBit); // Set Bit
            }
            else
            {
            	command[cmdByte] &= ~(1<<cmdBit);// Clear bit
            }

            // Move to next bit in the channel word
			chBit++;
			// If if that was the last bit, move to the next channel word
			if(chBit>10)
            {
				chBit = 0;
				chNdx++;
			}// if last channel bit
		}// for 8 bits in command byte
	}// for 7 command bytes, covers channels 1-5
	// Remaining channels are all set to 0x7FF
	// Set LSB of Channel 6 to 1
	command[6] |= 1<<7;
    // Set all command bytes to 0xFF
	for(cmdByte = 7; cmdByte < 22; cmdByte++)
    {
		command[cmdByte] = 0xFF;
	}
    // Send The Command
	while (!(IFG2&UCA0TXIFG));          // USCI_A0 TX buffer ready?
	UCA0TXBUF = 0x0F;			// Start Byte

	for(i = 0; i < 22; i++)
    {
    	while (!(IFG2&UCA0TXIFG));          // USCI_A0 TX buffer ready?
    	UCA0TXBUF = command[i];
    }
	while (!(IFG2&UCA0TXIFG));          // USCI_A0 TX buffer ready?
			UCA0TXBUF = 0x00;			// Flags Byte
	while (!(IFG2&UCA0TXIFG));          // USCI_A0 TX buffer ready?
			UCA0TXBUF = 0x00;			// Stop Byte
    return;
}

