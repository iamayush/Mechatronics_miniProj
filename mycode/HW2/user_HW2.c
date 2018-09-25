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
char torch = 0;
char recchar = 0;
int timecnt = 0;  
int timecheck = 0;
int valueIN = 0;
int mvIN = 0;
int lightsoffCNT = 0;

void main(void) {

	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	if (CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF) while(1);

	DCOCTL = CALDCO_16MHZ;    // Set uC to run at approximately 16 Mhz
	BCSCTL1 = CALBC1_16MHZ; 

	// Initializing ADC10
	ADC10CTL0 = SREF_0 + ADC10SHT_1 + ADC10ON + ADC10IE; // ADC10ON, interrupt enabled
	ADC10CTL1 = INCH_3 + ADC10DIV_0;                        // input A3                                              //PROBLEM 3
	ADC10AE0 |= BIT3;                         // A3 ADC option select                                                //PROBLEM 3

//	ADC10CTL1 = INCH_0 + ADC10DIV_0;                       // input A0                                               //PROBLEM 2
//	ADC10AE0 |= BIT0;                         // A0 ADC option select                                                //PROBLEM 2

	// Initialize Port 1
	P1SEL &= ~0xF0;  //                                                                                              //PROBLEM 3,5
	P1SEL2 &= ~0xF0; // P1.4 set as I/O                                                                              //PROBLEM 3,5
	P1REN = 0x0;  // No resistors enabled for Port 1                                                                 //
	P1DIR |= 0xF0; // Set P1.4 to output to drive LED                                                                //
	P1OUT &= ~0xF0;  // Initially set P1.4 to 0                                                                      //

	// Initialize Port 2                                                                                             //PROBLEM 4
	P2SEL |= 0x04;                                                                                                   //
	P2SEL2 &= ~0x04;                                                                                                 //
	P2DIR |= 0x04;                                                                                                   //

	// Timer A Config
	TACCTL0 = CCIE;       		// Enable Periodic interrupt
	TACCR0 = 16000;                // period = 1ms   
	TACTL = TASSEL_2 + MC_1; // source SMCLK, up mode

	// Timer A1 Config                                                                                               //PROBLEM 4
	TA1CTL = TASSEL_2 + MC_1;                                                                                        //PROBLEM 4
	TA1CCTL1 = OUTMOD_7;                                                                                             //
	TA1CCR0 = 1600;                                                                                                  //
	TA1CCR1 = 1600;                                                                                                  //


	Init_UART(9600,1);	// Initialize UART for 9600 baud serial communication

	_BIS_SR(GIE); 		// Enable global interrupt


	while(1) {

		if(newmsg) {
			newmsg = 0;
		}

		if(newprint) {
//			UART_printf("%d\n\r",mvIN);                                                                             //PROBLEM 2,3,4
			newprint = 0;
		}

//		if(torch){                                                                                                  //PROBLEM 3
//		    P1OUT |= 0x10;                                                                                          //
//		}                                                                                                           //
//		else{                                                                                                       //PROBLEM 3
//		    P1OUT &= ~0x10;                                                                                         //
//		}                                                                                                           //

	}
}


// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
	timecheck++; // Keep track of time for main while loop. 

	if (timecheck == 250) {
	    timecheck = 0;
	    timecnt++;
	    ADC10CTL0 |= ENC + ADC10SC; // Sampling and conversion start
	}

}



// ADC 10 ISR - Called when a sequence of conversions (A7-A0) have completed
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {

    valueIN = ADC10MEM;
    mvIN = (int)((valueIN*3300L)/1023);

    if(mvIN < 250){                                                                         //PROBLEM 3
        lightsoffCNT++;                                                                     //
    }                                                                                       //
    else{                                                                                   //PROBLEM 3
        lightsoffCNT = 0;                                                                   //
        torch = 0;                                                                          //
    }                                                                                       //
    if(lightsoffCNT >= 8){                                                                  //PROBLEM 3
        torch = 1;                                                                          //
    }                                                                                       //

    TA1CCR1 = (int)((valueIN*1600L)/1023);                                                  //PROBLEM 4

    newprint = 1;
}



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
	    recchar = (char)UCA0RXBUF;                                                          //PROBLEM 5
	    sendchar(recchar);                                                                  //PROBLEM 5
	    switch(recchar){                                                                    //
	    case '1':                                                                           //
	        P1OUT |= 0x10;                                                                  //
	        break;                                                                          //
	    case '2':
	        P1OUT |= 0x20;
	        break;
	    case '3':
	        P1OUT |= 0x40;
	        break;
	    case '4':
	        P1OUT |= 0x80;
	        break;
	    case '5':
	        P1OUT &= ~0x10;
	        break;
	    case '6':
	        P1OUT &= ~0x20;
	        break;
	    case '7':
	        P1OUT &= ~0x40;
	        break;                                                                          //
	    case '8':                                                                           //
	        P1OUT &= ~0x80;                                                                 //
	        break;                                                                          //
	    default:                                                                            //
	        P1OUT &= ~0xF0;                                                                 //PROBLEM 5
	    }                                                                                   //PROBLEM 5

		IFG2 &= ~UCA0RXIFG;
	}

}


