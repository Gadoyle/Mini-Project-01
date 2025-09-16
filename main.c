//Dylan Iden, Cannon Thomas, Garrett Doyle
//Mini Project

#include <msp430.h>

#define BTN1                BIT1        //P1.1
#define BTN2                BIT2        //P1.2
#define LMOTORF             BIT1        //P2.1, IN2, LEFT FORWARD
#define LMOTORB             BIT3        //P2.3, IN1, LEFT BACKWARDS
#define RMOTORF             BIT4        //P2.4, IN3, RIGHT FORWARDS
#define RMOTORB             BIT2        //P2.2, IN4, RIGHT BACKWARDS
#define COMP1               BIT0        //P3.0 COMPARATOR FOR SHUNT RESISTOR LMOTORF
#define COMP2               BIT1        //P3.1 COMPARATOR FOR SHUNT RESISTOR LMOTORB
#define COMP3               BIT2        //P3.2 COMPARATOR FOR SHUNT RESISTOR RMOTORF
#define COMP4               BIT3        //P3.3 COMPARATOR FOR SHUNT RESISTOR RMOTORB
#define LED                 BIT7        //P9.7 LED TO SIGNIFY DETECTED OC

void straight(){                        //Go straight
    P2OUT = (LMOTORF + RMOTORF);
}

void right(){                           //Turn
    P2OUT = (LMOTORF + RMOTORB);
}

void stop(){                            //hard stop
    P2OUT = (LMOTORF + LMOTORB + RMOTORF + RMOTORB);
}

void cruise(){                          //soft stop
    P2OUT = 0;//(LMOTORF + LMOTORB + RMOTORF + RMOTORB);
}

unsigned volatile int MEM = 0; // state changer between straight/turn
unsigned volatile short countA = 0;//the number that counts up in the PWM function, used for timing switch between straight/turn.
unsigned int OC = 0; // log overcurrent occurances
unsigned volatile int x = ; //SET TO VALUE REQUIRED TO TRAVEL 1 FOOT
unsigned volatile int y = ; //SET TO VALUE REQUIRED TO TURN 90 DEG.


void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	PM5CTL0 &= ~LOCKLPM5; //Unlock GPIO ports

    P2OUT &=~ (LMOTORF|LMOTORB|RMOTORF|RMOTORB);
    P2DIR |= (LMOTORF|LMOTORB|RMOTORF|RMOTORB);       //Motors initialized as outputs

    // LButton settup
	P1DIR &=~ (BIT1 + BIT2);
	P1REN |= (BIT1 + BIT2);
	P1OUT |= (BIT1 + BIT2);
	P1IE |= (BIT1 + BIT2);
	P1IES |= (BIT1 + BIT2);
	P1IFG = 0;
    
    // Comparator settup
	P3DIR &=~ (BIT0 + BIT1 + BIT2 + BIT3);
	P3REN |= (BIT0 + BIT1 + BIT2 + BIT3);
	P3OUT |= (BIT0 + BIT1 + BIT2 + BIT3);
	P3IE |= (BIT0 + BIT1 + BIT2 + BIT3);
	P3IES |= (BIT0 + BIT1 + BIT2 + BIT3);
	P3IFG = 0;
    
    P9DIR |= LED; // initializes LED as output. No pullup/down resistor req.

    //PWM
    TA2CTL = TASSEL__SMCLK + TBCLR + MC__STOP;      // PERIOD (SMCLK = 1MHz)
    TA2CCTL0 |= CCIE; 
    TA2CCR0 = 1000;                                 // DUTY CYCLE (1000/1MHz = 1ms, or 1kHz)

    countA = 0;                                     //Insures that there is no inconsistencies in our distance/timing.

    __enable_interrupt();
    __no_operation();

	while(1){}
}

#pragma vector = PORT1_VECTOR//turns on other timers when lbtn is pressed, effectively starting the race
__interrupt void MAXIMUMOVERDRIVE(void){
    if(P1IFG == BIT1) {
    TA1CTL |= MC__UP;
    TA2CTL |= MC__UP;
    P1IFG = 0;
}   else if (P1IFG == BIT2) {
    OC++;
    TA2CTL |= MC__UP;
    P9OUT ^= LED;
}
}

#pragma vector = PORT3_VECTOR
__interrupt void OVERCURRENT(void){
    TA2CTL |= MC__STOP;                             //Deactivates PWM, and therefor movement.
    P9OUT ^= LED;                                   //Toggles LED on board to show OC detected.
}  
#pragma vector = TIMER2_A0_VECTOR //PWM timer / Driver code
__interrupt void PWM_TIMER1(void) {
    countA++; //goes up every pulse to move motors and change states. One timer fits all.

    if (countA < x){ //go until x is reached                   
        straight();
    } 
    else  if ((countA >= x) && (countA < y)){ //turn once countA is past X and stop once hits Y
        right();
    } 
    else if (countA >= y){ //stop once countA hits or goes beyond Y due to clock jitter
        cruise();
        TA2CTL |= MC__STOP;
    }
}

