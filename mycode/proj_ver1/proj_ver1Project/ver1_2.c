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

#define PWM_FORWARD 475
#define PWM_BACKWARD 325
#define PWM_STOP 400
#define MAX_TURN 30
#define PWM_TURN_SPEED 100
#define TURN_TUNE_SPEED 60
#define TURN_TUNE_TIME 400
#define ONE_CELL_TIME 1500 // ms
#define LEFT_TURN_TIME 1200//1300 // ms-------------------------
#define PAUSE_CONTROL_TIME 1000 //ms
#define PAUSE_FORWARD 1000 //ms
#define TURN_TUNING_TOLERANCE 10 // IR units

//extern int explore_traj[35][2];

char newprint = 0;
int timecnt = 0;
int timecheck = 0;
int rightIRref_timer = 0;
int timerV = 0;
int cell_cnt = 0;


// Sensors
int sensors[8]; // array to hold ADC values
int photoR[20] = {0}; // photo-resistor for detecting occupied cell
int front_IR[20] = {0}; // IR proximity sensor in front
int right_IR[20] = {0}; // IR proximity sensor on right side
int right_IR_raw = 0; // not averaged value for ref calculation
int front_IR_raw = 0;
float right_IR_ref = 0; // averaged value used as reference for wall following I
int L_calls = 0;
int min_rightIR = 0;

//
int turn = 0;
char first_cmd = 1; // flags that program just started
char first_s_cmd = 0; // flags that car moves forward for the first time after stop
char dir_change = 1; // flags that direction of motion changed
int car_stop = 0; // flag to stop car when right IR ref is calculated
float K_turn = 0.2; // Proportional gain RW following
int fIR_preTurn = 0;
char drive_mode = 's';
int mode9_start = 0;

char move_type = 'x';
char start_timer = 0;

void drive_car(char mode){

    int left, right;
    switch(mode){
    case 's':
        //if (turn > 5*MAX_TURN) turn = 0; // ignore IR spikes
        //if (turn < -(5*MAX_TURN)) turn = 0;
        if (turn > MAX_TURN) turn = MAX_TURN;
        if (turn < -MAX_TURN) turn = -MAX_TURN;
        left = PWM_FORWARD - turn;
        right = PWM_FORWARD + turn;
        break;

    case 'l':
        left = PWM_STOP - PWM_TURN_SPEED;
        right = PWM_STOP + PWM_TURN_SPEED;
        break;

    case 'L': // turn right to tune 90 deg left turn
        left = PWM_STOP + TURN_TUNE_SPEED;
        right = PWM_STOP - TURN_TUNE_SPEED;
        break;

    case 'x':
        left = PWM_STOP;
        right = PWM_STOP;
        break;

    default:
        left = PWM_STOP;
        right = PWM_STOP;
    }

    // saturation
    if (left > PWM_FORWARD) left = PWM_FORWARD;
    if (left < PWM_BACKWARD)   left = PWM_BACKWARD;
    if (right > PWM_FORWARD) right = PWM_FORWARD;
    if (right < PWM_BACKWARD)   right = PWM_BACKWARD;

    TA1CCR1 = left;
    TA1CCR2 = right;
}

void main(void) {

    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

    if (CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF) while(1);

    DCOCTL = CALDCO_16MHZ;    // Set uC to run at approximately 16 Mhz
    BCSCTL1 = CALBC1_16MHZ;

    // Initialize ADC10
    // photoR at A7, front_IR at A6, right_IR at A5
    ADC10CTL1 = INCH_7 + ADC10SSEL_3 + CONSEQ_1; // Enable A7 first, Use SMCLK, Sequence of Channels
    ADC10CTL0 = ADC10ON + MSC + ADC10IE;  // Turn on ADC, Put in Multiple Sample and Conversion mode,  Enable Interrupt
    ADC10DTC1 = 8;                 // Eight conversions.
    ADC10SA = (short)&sensors[0];      // ADC10 data transfer starting address. Hence, array is filled backwards (i.e A7 in ADC[0] to A0 in ADC[7])

    // Initialize Port 2
    P2SEL |= 0x14;                                       // set P2.2 and P2.4 as
    P2SEL2 &= ~0x14;                                     // TA 1.1 and TA 1.2 for
    P2DIR |= 0x14;                                       // sending PWM to motors

    // Timer A Config
    TACCTL0 = CCIE;                                      // Enable Periodic interrupt
    TACCR0 = 16000;                                      // period = 1ms
    TACTL = TASSEL_2 + MC_1;                             // source SMCLK, up mode

    // Timer A1 Config
    TA1CTL = TASSEL_2 + MC_1;                     // SMCLK, up mode
    TA1CCTL0 = 0;                                        // corresponds to TA1CCR0
    TA1CCTL1 = OUTMOD_7;                                 // Reset/set mode for TA1.1 PWM
    TA1CCTL2 = OUTMOD_7;                                 // Reset/set mode for TA1.2 PWM
    TA1CCR0 = 800;                                     // carrier freq of 20 kHz
    TA1CCR1 = PWM_STOP;                             // initially robot at rest
    TA1CCR2 = PWM_STOP;

    //cell_position(); // get positions of all 36 cells in terms of IR sensor values
    //exploration(); // calculate exploration trajectory
    first_cmd = 1;
    dir_change = 1;

    Init_UART(9600,1);  // Initialize UART for 9600 baud serial communication

    _BIS_SR(GIE);       // Enable global interrupt

    while(1) {

        if(newmsg) {
            newmsg = 0;
        }

        if (newprint)  {

            //P1OUT ^= 0x1;     // Blink LED

            UART_printf("P %d F %d R %d\n\r",photoR[0],front_IR[0],right_IR[0]);

            newprint = 0;
        }

    }
}


// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
    timecheck++; // Keep track of time for main while loop.

    if (timecheck % 10 == 0){
        ADC10CTL0 |= ENC + ADC10SC;     // Enable Sampling and start ADC conversion
    }

    if (timecheck == 500) {
        timecheck = 0;
        timecnt++;
        //TA1CCR1 = PWM_FORWARD;
        //TA1CCR2 = PWM_FORWARD;
        newprint = 1;  // flag main while loop that .5 seconds have gone by.
    }
}



// ADC 10 ISR - Called when a sequence of conversions (A7-A0) have completed
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {

    if (start_timer == 1){
        timerV++;
        switch (move_type){
        case 'x':
            car_stop = 1;
            if ((timerV * 10) >= PAUSE_FORWARD){
                car_stop = 0;
                move_type = 's';
                timerV = 0;
                first_s_cmd = 1;
            }
            break;

        case 's':
            if ((timerV * 10) >= (cell_cnt * ONE_CELL_TIME)){
                //car_stop = 1;
                move_type = 'l';
                timerV = 0;
            }
            break;

        case 'l':
            if ((timerV * 10) >= LEFT_TURN_TIME){
                //dir_change = 1;
                move_type = 'L'; // tuning 90 left turn
                min_rightIR = 2000;
                L_calls = -1;
                timerV = 0;
            }
            break;

        case 'L':
            L_calls++;
            if (right_IR[0] < min_rightIR){
                min_rightIR = right_IR[0];
            }
            if ((right_IR[0] > (min_rightIR + TURN_TUNING_TOLERANCE)) && (L_calls >= 10)){
                dir_change = 1;
                timerV = 0;
            }
            if ((L_calls * 10) >= TURN_TUNE_TIME){
                dir_change = 1;
                timerV = 0;
            }
            break;

        default:
            car_stop = 1;
        }

    }

    //int test = explore_traj[0][0];

    photoR[0] = sensors[0];
    front_IR[0] = sensors[1];
    right_IR[0] = sensors[2];

    // Reversing IR values to get high value for larger distance
    front_IR[0] = 1023 - front_IR[0];
    right_IR[0] = 1023 - right_IR[0];

    // Not averaged raw value for right_IR_ref calculations
    right_IR_raw = right_IR[0];
    front_IR_raw = front_IR[0];

    // average filtering sensor data
    int i = 19; // 20 old vals used in filtering
    for (i = 19; i > 0; i--){
        photoR[0] += photoR[i];
        front_IR[0] += front_IR[i];
        right_IR[0] += right_IR[i];

        if (i > 1){
            photoR[i] = photoR[i-1];
            front_IR[i] = front_IR[i-1];
            right_IR[i] = right_IR[i-1];
        }
    }

    photoR[0] = ((float)photoR[0])/20.0;
    front_IR[0] = ((float)front_IR[0])/20.0;
    right_IR[0] = ((float)right_IR[0])/20.0;

    photoR[1] = photoR[0];
    front_IR[1] = front_IR[0];
    right_IR[1] = right_IR[0];


    // find reference right_IR when dir is changed
    if (dir_change){
        car_stop = 1;
        start_timer = 0;
        if  (rightIRref_timer == 0) right_IR_ref = 0;

        rightIRref_timer++;

        right_IR_ref += (float)right_IR_raw;

        if (rightIRref_timer >= 200){
            right_IR_ref = right_IR_ref/200.0;
            rightIRref_timer = 0;
            //mode9_start = 0;
            move_type = 's';
            timerV = 0;
            start_timer = 1; // start timerV
            cell_cnt = 3; // no. of cells to be traveled
            car_stop = 0;
            dir_change = 0;
            if (first_cmd){
                move_type = 'x';
                car_stop = 1;
                first_cmd = 0;
            }
        }
    }

    // obstacle detection in front
    if ((front_IR_raw < 380) && (start_timer == 1)){
        move_type = 'l';
        timerV = 0;
    }

    /****************************************************************************************/
    if (move_type == 's'){
        // turn calculated as control input for right wall following
        turn = (int)(K_turn*(right_IR_ref - right_IR[0]));

        // Motor switching ON causes weird IR vals
        if ((timerV*10 < PAUSE_CONTROL_TIME) && (first_s_cmd)) {
            turn = 0;
            if (timerV*10 >= PAUSE_CONTROL_TIME - 10)   first_s_cmd = 0;
        }
        drive_mode = 's'; // straight driving
    }

    if (move_type == 'l'){
        drive_mode = 'l'; // left turn
    }

    if (move_type == 'L'){
        drive_mode = 'L'; // right-turn for tuning 90 deg left turn
    }
    //    if ( (((front_IR_raw - 0) < 10) || ((0 - front_IR_raw) < 10)) && (mode9_start == 0) && (car_stop == 0)){
    //        fIR_preTurn = 380;
    //        mode9_start = 1; // flag that turn has started
    //    }
    //
    //    if (mode9_start){
    //        drive_mode = '9'; // turn by 90 degrees
    ////        if ( (right_IR[0] - fIR_preTurn) < 10 || (fIR_preTurn - right_IR[0] < 10) ){
    ////            dir_change = 1;
    ////            mode9_start = 0;
    ////        }
    //    }

    /****************************************************************************************/
    // stop car: highest preference so last statement before sending command
    if (car_stop){
        drive_mode = 'x';
    }

    // send robot drive commands
    drive_car(drive_mode); // modes as input: forward, 90 deg turn etc.

    // for next call of ADC ISR
    ADC10CTL0 &= ~ADC10IFG;  // clear interrupt flag
    ADC10SA = (short)&sensors[0]; // ADC10 data transfer starting address
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

    if(IFG2&UCB0TXIFG) {    // USCI_B0 requested TX interrupt (UCB0TXBUF is empty)

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

