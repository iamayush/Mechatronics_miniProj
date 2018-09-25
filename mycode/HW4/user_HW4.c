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
int cnt2_6 = 0;
int cnt2_7 = 0;
char flag2_6 = 0;
char flag2_7 = 0;
int time2_6 = 0;
int time2_7 = 0;

void main(void) {

    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

    if (CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF) while(1);

    DCOCTL = CALDCO_16MHZ;    // Set uC to run at approximately 16 Mhz
    BCSCTL1 = CALBC1_16MHZ;

    //    // Initialize Port 1
    //    P1SEL &= ~0x01;  // See page 42 and 43 of the G2553's datasheet, It shows that when both P1SEL and P1SEL2 bits are zero
    //    P1SEL2 &= ~0x01; // the corresponding pin is set as a I/O pin.  Datasheet: http://coecsl.ece.illinois.edu/ge423/datasheets/MSP430Ref_Guides/msp430g2553datasheet.pdf
    //    P1REN = 0x0;  // No resistors enabled for Port 1
    //    P1DIR |= 0x1; // Set P1.0 to output to drive LED on LaunchPad board.  Make sure shunt jumper is in place at LaunchPad's Red LED
    //    P1OUT &= ~0x01;  // Initially set P1.0 to 0


    // Initialize Port 2
    P2SEL &= ~0xC3;
    P2SEL2 &= ~0xC3;
    P2DIR &= ~0xC0;
    P2DIR |= 0x03;
    P2REN |= 0xC0;
    P2OUT |= 0xC3;

    // Port 2 Interrupts
    P2IE |= 0xC0;
    P2IES |= 0xC0;
    P2IFG &= ~0xC0;

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
            UART_printf("P2.6 %d P2.7 %d\n\r",cnt2_6,cnt2_7); // divide by two so displays seconds by default
            newprint = 0;
        }
    }
}


// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
    timecheck++; // Keep track of time for main while loop.
    time2_6++;
    time2_7++;

    if (timecheck == 500) {
        timecheck = 0;
        timecnt++;
        newprint = 1;  // flag main while loop that .5 seconds have gone by.
    }

    if (time2_6 == 10){
        time2_6 = 0;
        if (flag2_6){
            flag2_6 = 0;
            P2IE |= 0x40;
        }
    }

    if (time2_7 == 10){
        time2_7 = 0;
        if (flag2_7){
            flag2_7 = 0;
            P2IE |= 0x80;
        }
    }

}

// Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    if ((P2IFG & 0x40) == 0x40){
        cnt2_6++;
        P2OUT ^= 0x01;                            // P2.0 toggled
        P2IFG &= ~0x40;                           // P1.4 IFG cleared
        time2_6 = 0;
        flag2_6 = 1;
    }
    if ((P2IFG & 0x80) == 0x80){
        cnt2_7++;
        P2OUT ^= 0x02;                            // P2.1 toggled
        P2IFG &= ~0x80;                           // P1.4 IFG cleared
        time2_7 = 0;
        flag2_7 = 1;
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


