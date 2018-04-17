// Includes
#include <msp430.h>
#include "i2cslave.h"
#include <msp430g2553.h>                // include necessary templates
#include <intrinsics.h>
#include <stdint.h>


// Define I/O Pins, Trigger on P1.x and Echo on P2.x
#define ECHO0 BIT0                          //configure Pins
#define TRIG0 BIT0
#define ECHO1 BIT1                          //configure Pins
#define TRIG1 BIT1
#define ECHO2 BIT2                          //configure Pins
#define TRIG2 BIT2
#define ECHO3 BIT3                          //configure Pins
#define TRIG3 BIT3
#define ECHO4 BIT4                          //configure Pins
#define TRIG4 BIT4
#define ECHO5 BIT5                          //configure Pins
#define TRIG5 BIT5
// Define Ports being used
#define TRIGPORT P1DIR
#define TRIGOUT P1OUT
#define ECHOPORT P2DIR
#define ECHOSEL  P2SEL
#define N_SENSE 6						// Number of Sensors
#define BOARD_ADDRESS 0x52
#define ECHO_DELAY 9600					//
#define ECHO_DELAY_MULIPLIER 100
#define TRIG_DELAY 160
#define CM_PER_CYCLE 1/131.0

int input_handler (char *instring, char *outstring);

volatile float distance_in_cm=0;   // set up integers for measuring
float distance[6] = {0, 0, 0, 0, 0, 0};
volatile unsigned int start_time;
volatile unsigned int total_time;
volatile unsigned int up=0;
volatile unsigned int sensorIdx = 0;

void init (void)
{
	WDTCTL = WDTPW | WDTHOLD;			//kill watchdog timer

	__delay_cycles(50000);
	i2c_slave_init(BOARD_ADDRESS); 		//Set slave address
	__delay_cycles(50000);
	BCSCTL1 = CALBC1_16MHZ;				// Set Clock Speed to 16MHz
	DCOCTL = CALDCO_16MHZ;
	volatile int k;
	for (k=0;k<200;k++)
		__delay_cycles(50000);


	TA1CTL = TASSEL_2 + ID_3 + MC_2;	// configure timer A1

	// Port Setup
	TRIGPORT |= TRIG0 | TRIG1 | TRIG2 | TRIG3 | TRIG4 | TRIG5;		// set trigger as output
	ECHOPORT &= ~(ECHO0 | ECHO1 | ECHO2 | ECHO3 | ECHO4 | ECHO5);	// echos as inputs
	ECHOSEL &= ~(ECHO0 | ECHO1 | ECHO2 | ECHO3 | ECHO4 | ECHO5);	// set no pins to (capture mode), yet

}

void main(void)
{
	init();
	__enable_interrupt();                                     // enabling interrupts

	volatile int ok2send=0;

	i2cTXData[0]=1;						//Dummy values first input to i2cTXData
	i2cTXData[1]=2;
	i2cTXData[2]=3;
	i2cTXData[3]=3;
	i2cTXData[4]=3;
	i2cTXData[5]=3;

	while (1)
	{
		if (i2crxflag){					// if data received as the I2C slave
			__delay_cycles(50000);
			ok2send=input_handler(i2cRXData+1,i2cTXData);
			i2crxflag=0;
		}

		switch(sensorIdx){
		case 0:
			TA1CCTL0 = CM_3 | SCS | CAP | CCIE | CCIS_0;	// Changes which pin triggers the interrupt
			ECHOSEL = ECHO0;								// Set current echo pin to Timer Capture Mode
			up = 1;                                        	// when pulse is send, up =1 (high edge),  TACCR0 detects rising and falling edges
			TRIGOUT |= TRIG0; 			                   	// sending pulses for the HC-SR04
			__delay_cycles(TRIG_DELAY);
			TRIGOUT &=~ TRIG0;				// wait long enough for the ISR to kick in
			break;
		case 1:
			TA1CCTL1 = CM_3 | SCS | CAP | CCIE | CCIS_0;
			ECHOSEL = ECHO1;
			up = 1;
			TRIGOUT |= TRIG1;
			__delay_cycles(TRIG_DELAY);
			TRIGOUT &=~ TRIG1;
			break;
		case 2:
			TA1CCTL1 = CM_3 | SCS | CAP | CCIE | CCIS_1;
			ECHOSEL = ECHO2;
			up = 1;
			TRIGOUT |= TRIG2;
			__delay_cycles(TRIG_DELAY);
			TRIGOUT &=~ TRIG2;
			break;
		case 3:
			TA1CCTL0 = CM_3 | SCS | CAP | CCIE | CCIS_1;
			ECHOSEL = ECHO3;
			up = 1;
			TRIGOUT |= TRIG3;
			__delay_cycles(TRIG_DELAY);
			TRIGOUT &=~ TRIG3;
			break;
		case 4:
			TA1CCTL2 = CM_3 | SCS | CAP | CCIE | CCIS_0;
			ECHOSEL = ECHO4;
			up = 1;
			TRIGOUT |= TRIG4;
			__delay_cycles(TRIG_DELAY);
			TRIGOUT &=~ TRIG4;
			break;
		case 5:
			TA1CCTL2 = CM_3 | SCS | CAP | CCIE | CCIS_1;
			ECHOSEL = ECHO5;
			up = 1;
			TRIGOUT |= TRIG5;
			__delay_cycles(TRIG_DELAY);
			TRIGOUT &=~ TRIG5;
			break;
		default:
			break;
		}

		volatile int k;
			for (k=0;k<ECHO_DELAY_MULIPLIER;k++)
				__delay_cycles(ECHO_DELAY);

		sensorIdx = 1;
		if(sensorIdx > 5)
			sensorIdx = 0;


	}
}

//------------------------------------------------------
#pragma vector=TIMER1_A0_VECTOR
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMERA1_ISR (void)
{
	if(sensorIdx == 0 || sensorIdx == 3){
		if (up){
			start_time = TA1CCR0;				// check for high edge, if so, TACCR0 is the start time
		}
		else {
			total_time = TA1CCR0 - start_time;	// when up = 0, your time the pulse went and came back is TACCR0- start time
			distance_in_cm = total_time*CM_PER_CYCLE;
			distance[sensorIdx] = distance_in_cm;
		}
	}else if(sensorIdx == 1 || sensorIdx == 2){
		if (up){
			start_time = TA1CCR1;
		}
		else {
			total_time = TA1CCR1 - start_time;
			distance_in_cm = total_time*CM_PER_CYCLE;
			distance[sensorIdx] = distance_in_cm;
		}
	}else if(sensorIdx == 4 || sensorIdx == 5){
		if (up){
			start_time = TA1CCR2;
		}
		else {
			total_time = TA1CCR2 - start_time;
			distance_in_cm = total_time*CM_PER_CYCLE;
			distance[sensorIdx] = distance_in_cm;
		}
	}
	TA1CTL &= ~TAIFG;
	TA1CCTL0 &= ~CCIFG;					// clear CCI flag
	TA1CCTL1 &= ~CCIFG;
	TA1CCTL2 &= ~CCIFG;
	up=!up;			// after this, up needs to be opposite as it was
}


int input_handler (char *instring, char *outstring){

	switch(instring[0]){

		case 'A':
			outstring[0]= distance[0];
			outstring[1]= distance[1];
			outstring[2]= distance[2];
			outstring[3]= distance[3];
			outstring[4]= distance[4];
			outstring[5]= distance[5];
			break;
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
			outstring[0]= distance[instring[0]-0x30];
			break;
		default:
			return 0;
	}

	return 1;
}
