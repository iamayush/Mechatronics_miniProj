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

char newprint = 0; // flags main to print every 500ms
int timecnt = 0;   // keeps time
int timecheck = 0; // keeps time
int timeRC1 = 0;   // keeps time for RC servo 1
int timeRC2 = 0;   // keeps time for RC servo 2
int RC1flag = 0;   // flags RC servo1's position
int RC2flag = 0;   // flags RC servo2's position
int DACVAL = 0;    // 10 bit value to be sent to DAC
int DACSEND = 0;   // 16 bits to be sent to DAC
int firstTX = 0;   // flags whether first 8 bits of DACSEND were sent or not
int ramp = 0;      // ramp signal
int valueIN = 0;   // value from ADC10MEM (converted photresistor Voltage)


void main(void) {

    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

    if (CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF) while(1);

    DCOCTL = CALDCO_16MHZ;    // Set uC to run at approximately 16 Mhz
    BCSCTL1 = CALBC1_16MHZ;

    // Initializing ADC10
    ADC10CTL0 = SREF_0 + ADC10SHT_1 + ADC10ON + ADC10IE; // ADC10ON, interrupt enabled
    ADC10CTL1 = INCH_3 + ADC10DIV_0;                     // input A3
    ADC10AE0 |= BIT3;

    P1OUT = 0x00;                                        // P1 set to 0 initially
    P1DIR |= BIT6;                                       // set P1.6 as output for FS pulse
    P1SEL = BIT5 + BIT7;                                 // config for using P1.5 and P1.7
    P1SEL2 = BIT5 + BIT7;                                // as UCB0CLK and UCB0SIMO
    UCB0CTL0 = UCCKPL + UCCKPH + UCMSB + UCMST + UCSYNC; // 3-pin,synchronous,8-bit,master, polarity and phase 1
    UCB0CTL1 = UCSSEL_2 + UCSWRST;                       // SMCLK and set in SW reset
    UCB0BR0 = 80;                                        // set baud rate 80 cycles per cycle of SMCLK
    UCB0BR1 = 0;
    UCB0CTL1 &= ~UCSWRST;                                // **Initialize USCI state machine**
    IFG2 &= ~UCB0TXIE;                                   // enable interrupts for
    IE2 |= UCB0TXIE;                                     // UCB0TX

    // Initialize Port 2
    P2SEL |= 0x12;                                       // set P2.1 and P2.4 as
    P2SEL2 &= ~0x12;                                     // TA 1.1 and TA 1.2 for
    P2DIR |= 0x12;                                       // sending PWM to servos

    // Timer A Config
    TACCTL0 = CCIE;                                      // Enable Periodic interrupt
    TACCR0 = 16000;                                      // period = 1ms
    TACTL = TASSEL_2 + MC_1;                             // source SMCLK, up mode

    // Timer A1 Config
    TA1CTL = TASSEL_2 + MC_1 + ID_3;                     // SMCLK, up mode, divide by 8
    TA1CCTL0 = 0;                                        // corresponds to TA1CCR0
    TA1CCTL1 = OUTMOD_7;                                 // Reset/set mode for TA1.1 PWM
    TA1CCTL2 = OUTMOD_7;                                 // Reset/set mode for TA1.2 PWM
    TA1CCR0 = 40000;                                     // carrier freq of 50 Hz
    TA1CCR1 = 2000;                                      // initialize dc of pwm1 to 5%
    TA1CCR2 = 2000;                                      // initialize dc of pwm2 to 5%


    Init_UART(9600,1);                                   // Initialize UART for 9600 baud serial

    _BIS_SR(GIE);                                        // Enable global interrupt


    while(1) {

        if(newmsg) {
            newmsg = 0;
        }

        if (newprint)  {
            //UART_printf("%d\n\r",TA1CCR1);
            newprint = 0;
        }
    }
}


// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
    ADC10CTL0 |= ENC + ADC10SC;                     // Sampling and conversion start every 1ms
}


// ADC 10 ISR - Called when a sequence of conversions (A7-A0) have completed
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {

    valueIN = ADC10MEM;                            // store value from photo-resistor
    //DACVAL = ramp;                               // ramp from 0 to 1023
    DACVAL = valueIN;                              // DACVAL has 10 bit to be sent
    DACSEND = ((DACVAL<<2) & 0x0FFF) | 0x4000;     // DACSEND has 16 bits= bit0,1 as zero, bit2-11 as DACVAL
                                                   // and bits12-15 for fast mode and normal power
    P1OUT |= 0x40;                                 // Frame Select Pulse
    P1OUT &= ~0x40;                                // at P1.6
    UCB0TXBUF = DACSEND>>8;                        // send Most significant 8 bits
    firstTX = 1;                                   // flag as 1st 8bits handled
    ramp++;                                        // incrementing ramp
    if(ramp>1023){                                 // roll back ramp to zero on reaching max
        ramp = 0;
    }

    // RC1 position toggle every 1s
    timeRC1++;                                     // Keep track of time for main while loop.
    if (timeRC1 == 1000) {
        timeRC1 = 0;
        if(RC1flag == 1){
            TA1CCR1 = 2000;                        // set at 5% position
            RC1flag = 0;                           // flags that servo at 5% position
        }
        else{
            TA1CCR1 = 4000;                        // set at 10% position
            RC1flag = 1;                           // flags that servo at 10% position
        }
    }

    // RC2 position toggle every 1s
    timeRC2++;                                     // Keep track of time for main while loop.
    if (timeRC2 == 1000) {
        timeRC2 = 0;
        if(RC2flag == 1){                          // same as RC servo 1
            TA1CCR2 = 2000;
            RC2flag = 0;
        }
        else{
            TA1CCR2 = 4000;
            RC2flag = 1;
        }
    }

    // prints every 500 ms
    timecheck++; // Keep track of time for main while loop.
    if (timecheck == 500) {
        timecheck = 0;
        timecnt++;
        newprint = 1;  // flag main while loop that .5 seconds have gone by.
    }

}



// USCI Transmit ISR - Called when TXBUF is empty (ready to accept another character)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {

    if(IFG2&UCA0TXIFG) {        // USCI_A0 requested TX interrupt
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

    if(IFG2&UCB0TXIFG) {                        // USCI_B0 requested TX interrupt (UCB0TXBUF is empty)
        if(1 == firstTX){                       // enter only if MSB 8 bits were already sent
        UCB0TXBUF = DACSEND;                    // send LSB 8 bits
        firstTX = 0;                            // flag that 1st Tx hasn't been done yet
        }
        IFG2 &= ~UCB0TXIFG;                     // clear IFG
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
/*      if(!started) {  // Haven't started a message yet
            if(UCA0RXBUF == 253) {
                started = 1;
                newmsg = 0;
            }
        }
        else {  // In process of receiving a message
            if((UCA0RXBUF != 255) && (msgindex < (MAX_NUM_FLOATS*5))) {
                rxbuff[msgindex] = UCA0RXBUF;

                msgindex++;
            } else {    // Stop char received or too much data received
                if(UCA0RXBUF == 255) {  // Message completed
                    newmsg = 1;
                    rxbuff[msgindex] = 255; // "Null"-terminate the array
                }
                started = 0;
                msgindex = 0;
            }
        }
*/

        IFG2 &= ~UCA0RXIFG;
    }

}
