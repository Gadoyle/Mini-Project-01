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

unsigned volatile int MEM = 0; // remembers the previous drive mode
unsigned volatile short countA=0;//the number that counts up in the PWM function
unsigned const short PWM = BIT3;//the PWM output is used to set variables to stop. although this is a remenant from a previous version that used port interupts
unsigned volatile short PWM_COUNTER = 6;//the default number that affects when the PWM is on or off
unsigned int OC = 0; // log overcurrent occurances


void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	PM5CTL0 &= ~LOCKLPM5; //Unlock GPIO ports

    P2OUT &=~ (LMOTORF|LMOTORB|RMOTORF|RMOTORB);
    P2DIR |= (LMOTORF|LMOTORB|RMOTORF|RMOTORB);       //Motors initialized as outputs

    //PWM and IR sensor setup
    P3DIR |= PWM;
    P3OUT &=~ PWM;
	P3IFG =0;
	
    // LButton settup
	P1DIR &=~ BIT1;
	P1REN |= BIT1;
	P1OUT |= BIT1;
	P1IE |= BIT1;
	P1IES |= BIT1;
	P1IFG = 0;
    
    // Comparator settup
	P3DIR &=~ (BIT0 + BIT1 + BIT2 + BIT3);
	P3REN |= (BIT0 + BIT1 + BIT2 + BIT3);
	P3OUT |= (BIT0 + BIT1 + BIT2 + BIT3);
	P3IE |= (BIT0 + BIT1 + BIT2 + BIT3);
	P3IES |= (BIT0 + BIT1 + BIT2 + BIT3);
	P3IFG = 0;

    //Timer for distance
    TA1CTL = TASSEL__SMCLK + TACLR + MC__STOP;
    TA1CCTL0 |= CCIE;
    TA1CCR0 = 1000; // SET UP TIME FOR TIMER IN ADVANCE

    
    //PWM
    TA2CTL = TASSEL__SMCLK  + TACLR + MC__STOP;
    TA2CCTL0 |= CCIE;
    TA2CCR0 = 1000;





    __enable_interrupt();
    __no_operation();
    //int scroll =0;

	while(1){}
}

#pragma vector = PORT1_VECTOR//turns on other timers when lbtn is pressed, effectively starting the race
__interrupt void MAXIMUMOVERDRIVE(void){
    if(P1IFG = BIT1) {
    TA1CTL |= MC__UP;
    TA2CTL |= MC__UP;
    P1IFG = 0;
}   else if (P1IFG = BIT2) {
    OC += OC;
    
}
}

#pragma vector = PORT3_VECTOR
__interrupt void OVERCURRENT(void){
    
}
#pragma vector = TIMER2_A0_VECTOR //PWM timer / Driver code
__interrupt void PWM_TIMER1(void) {

    if ((countA==PWM_COUNTER)){ // THE HIGHER PWM_COUNTER, THE SLOWER
        P3OUT &=~ PWM;


    }if(countA==9){// COUNTS TO THIS VALUE, THEN RESETS EXCLUDING THE COUNT UP INITIALLY
        P3OUT |= PWM;
        countA = 0;
    }else{
        countA++;
    }

    else if (/*(!(ctrl[0]))&&(!ctrl[1])&&(!(ctrl[2]))*/){ //SET THIS UP WITH TIMER IN ADVANCE, USE DIFFERENT VARIABLES FOR DISTANCE MEASUREMENT.
        straight();
        MEM = P2OUT;
        PWM_COUNTER = 0;

    }
    }else  if (/*(!ctrl[0])&&(!ctrl[1])&&((ctrl[2]))*/){//hard right
        hright();
        MEM = P2OUT;
        PWM_COUNTER = 4;

}
