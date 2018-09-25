/******************************************************************************
MSP430G2553 Project Creator

GE 423  - Dan Block
        Spring(2017)

        Written(by) : Steve(Keres)
College of Engineering Control Systems Lab
University of Illinois at Urbana-Champaign
*******************************************************************************/

#include "msp430g2553.h"
#include "UART.h"

char newprint = 0;
int timecnt = 0;  
int timecheck = 0;  
char switchstate = '0'; //variable indicating status of both switches

/*Function returning status of both switches*/
char get_switchstate(void){
    switch ((~P2IN & 0x80)|(~P2IN & 0x40)){ //~P2IN has 1 at bit7 when switch 1 is pressed and has 1 at bit6 when switch 2 is pressed
    case 0x0 :
        return '0'; //no switch pressed
    case 0x80 :
        return '1'; //only switch 1 pressed
    case 0x40 :
        return '2'; //only switch 2 pressed
    case 0xC0 :
        return '3'; //both switches pressed
    }
    return '0';
}

void main(void) {

	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	if (CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF) while(1);

	DCOCTL = CALDCO_16MHZ;    // Set uC to run at approximately 16 Mhz
	BCSCTL1 = CALBC1_16MHZ; 

	// Initialize Port 1
	P1SEL &= ~0xF1;  // pins 0,4,5,6,7 are
	P1SEL2 &= ~0xF1; //set as a I/O pins
	P1REN = 0x0;  // No resistors enabled for Port 1
	P1DIR |= 0xF1; // Set P1.0,P1.4-P1.7 as output pins
	P1OUT &= ~0xF1;  // Initially set output pins to 0

	// Initialize Port 2
	P2SEL &= ~0xC0;  // pins 6 and 7 are set
	P2SEL2 &= ~0xC0; // as I/O pins
	P2REN |= 0xC0;  // Resistors enabled for P2.6,P2.7
	P2DIR &= ~0xC0; // Set P2.6, P2.7 as input pins
	P2OUT |= 0xC0;  // Pull up for Pins 6,7

	// Timer A Config
	TACCTL0 = CCIE;       		// Enable Periodic interrupt
	TACCR0 = 16000;                // period = 1ms   
	TACTL = TASSEL_2 + MC_1; // source SMCLK, up mode


	Init_UART(9600,1);	// Initialize UART for 9600 baud serial communication

	_BIS_SR(GIE); 		// Enable global interrupt


	while(1) {

		if(newmsg) {
			newmsg = 0;
		}

		if (newprint)  {

			//P1OUT ^= 0x1;		// Blink LED

			//UART_printf("Hello %d\n\r",timecnt/2); // divide by two so displays seconds by default

			newprint = 0;
		}

		switchstate = get_switchstate(); //switch status updates continuously
		switch(switchstate){ //print status of switches
		case '0' :
		    P1OUT = 0x0;
		    UART_printf("No switch pressed\n\r");
		    break;
		case '1' :
		    P1OUT = 0x10;
		    UART_printf("Switch 1 is pressed\n\r");
		    break;
		case '2' :
		    P1OUT = 0x20;
		    UART_printf("Switch 2 is pressed\n\r");
		    break;
		case '3' :
		    P1OUT = 0x30;
		    UART_printf("Both switches are pressed\n\r");
		    break;
		}

	}
}


// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
	timecheck++; // Keep track of time for main while loop. 

	if (timecheck == 500) {
	    timecheck = 0;
	    timecnt++;
	    newprint = 1;  // flag main while loop that .5 seconds have gone by.  
	}

}


/*
// ADC 10 ISR - Called when a sequence of conversions (A7-A0) have completed
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {

}
*/


// USCI Transmit ISR - Called when TXBUF is empty (ready to accept another character)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {

	if(IFG2&UCA0TXIFG) {		// USCI_A0 requested TX interrupt
		if(printf_flag) {
			if (currentindex == txcount) {
				senddone = 1;
				printf_flag = 0;
				IFG2 &= ~UCA0TXIFG;
			} else {
				UCA0TXBUF = printbuff[currentindex];
				currentindex++;
			}
		} else if(UART_flag) {
			if(!donesending) {
				UCA0TXBUF = txbuff[txindex];
				if(txbuff[txindex] == 255) {
					donesending = 1;
					txindex = 0;
				}
				else txindex++;
			}
		} else {  // interrupt after sendchar call so just set senddone flag since only one char is sent
			senddone = 1;
		}

		IFG2 &= ~UCA0TXIFG;
	}

	if(IFG2&UCB0TXIFG) {	// USCI_B0 requested TX interrupt (UCB0TXBUF is empty)

		IFG2 &= ~UCB0TXIFG;   // clear IFG
	}
}


// USCI Receive ISR - Called when shift register has been transferred to RXBUF
// Indicates completion of TX/RX operation
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {

	if(IFG2&UCB0RXIFG) {  // USCI_B0 requested RX interrupt (UCB0RXBUF is full)

		IFG2 &= ~UCB0RXIFG;   // clear IFG
	}

	if(IFG2&UCA0RXIFG) {  // USCI_A0 requested RX interrupt (UCA0RXBUF is full)

//    Uncomment this block of code if you would like to use this COM protocol that uses 253 as STARTCHAR and 255 as STOPCHAR
/*		if(!started) {	// Haven't started a message yet
			if(UCA0RXBUF == 253) {
				started = 1;
				newmsg = 0;
			}
		}
		else {	// In process of receiving a message		
			if((UCA0RXBUF != 255) && (msgindex < (MAX_NUM_FLOATS*5))) {
				rxbuff[msgindex] = UCA0RXBUF;

				msgindex++;
			} else {	// Stop char received or too much data received
				if(UCA0RXBUF == 255) {	// Message completed
					newmsg = 1;
					rxbuff[msgindex] = 255;	// "Null"-terminate the array
				}
				started = 0;
				msgindex = 0;
			}
		}
*/

		IFG2 &= ~UCA0RXIFG;
	}

}


