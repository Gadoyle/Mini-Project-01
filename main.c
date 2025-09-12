//Dylan Iden, Sam Milford, Joshua Spatafore
//Final Project
/*This project is for a line following robot that senses lines with IR sensors and detects objects with ultrasound
 *  The robot will stop if it detects any object less than 14 cm from the ultrasound.
 *  The robot also uses a PWM function to improve driving and steering performance.
 *  The robot displays distance from the IR sensor to the LCD display.
 *  Won 1st place in a rover bracket in terms of speed and reliability.
 *Coded only in interrupts
 *  Have a great summer Mr. Tarter! Good luck on your doctorate stuff.
 */

#include <msp430.h>

#define LMOTORF             BIT1        //P2.1, IN2, LEFT FORWARD
#define LMOTORB             BIT3        //P2.3, IN1, LEFT BACKWARDS
#define RMOTORF             BIT4        //P2.4, IN3, RIGHT FORWARDS
#define RMOTORB             BIT2       //P2.2, IN4, RIGHT BACKWARDS
#define LSENS               BIT1        //P3.1    LEFT = 1
#define CSENS               BIT0        //P3.0    CENTER = 0
#define RSENS               BIT2        //P3.2    RIGHT = 2
#define BTN1                BIT1        //P1.1

void straight(){        //either center or all three
    //P2OUT &=~ (LMOTORB + RMOTORB);
    P2OUT = (LMOTORF + RMOTORF);
}
void hleft(){          //just left, right motor full back left motor full front
    //P2OUT &=~ (LMOTORF + RMOTORB);
    P2OUT = (LMOTORB + RMOTORF);
}
void hright(){           //just right, opposite
    //P2OUT &=~ (LMOTORB + RMOTORF);
    P2OUT = (LMOTORF + RMOTORB);
}
void sright(){          //center & left, only right motor
    //P2OUT &=~ (RMOTORF + RMOTORB + LMOTORB);
    P2OUT = (LMOTORF);
}
void sleft(){           //center & right, only left motor
    //P2OUT &=~ (LMOTORF + LMOTORB + RMOTORB);
    P2OUT = (RMOTORF);
}
void stop(){            //hard stop
    P2OUT = (LMOTORF + LMOTORB + RMOTORF + RMOTORB);
}
void cruise(){          //soft stop
    P2OUT = 0;//(LMOTORF + LMOTORB + RMOTORF + RMOTORB);
}

/**
 * Code route following
 * //a button that will enable the interrupts for the other funcs
 */
unsigned volatile int MEM = 0; // remembers the previous drive mode
unsigned volatile short lft,ctr,rht=0; // variables for saving input from IR
unsigned volatile short countA=0;//the number that counts up in the PWM function
unsigned const short PWM = BIT3;//the PWM output is used to set variables to stop. although this is a remenant from a previous version that used port interupts
unsigned volatile short PWM_COUNTER = 6;//the default number that affects when the PWM is on or off
unsigned const int TRIG = 0X02;// 4.1 defining trigger output
unsigned volatile int distance = 100;// Distance for Ultra Sound
unsigned const int ECHO = 0X04;// 4.2 defining echo input

unsigned char TXBuf[32]; // buffer for holding data to be sent
unsigned char *TXPtr = TXBuf; // pointer for TXBuf
unsigned char sendR = 0;//step counter for returning
unsigned char sendB = 0;//step counter for backspace
unsigned char backspace[4] = {'\b',' ','\b',0};// backspace sequence
//My personal LCD display library, numbers from 0-9 and capital letters only
unsigned const int libL [] = {0xFC,0x60,0xDB,0xF3,0x67//4
                              ,0xB7,0xBF,0xE0,0xFF,0xE7//9
                              ,0xEF,0xF1,0x9C,0xF0,0x9F,0x8F//F
                              ,0x9D,0x6F,0X90,0X78,0X00,0X1C//L
                              ,0XEC,0X6C,0XFC,0XCF,0XFC,0XCF//R
                              ,0XB7,0X80,0X7C,0X0C,0X7C,0X00//X
                              ,0X47,0X90//Z
                              ,0x00};//nothing (36)
unsigned const int libH [] = {0x28,0x00,0x00,0x00,0x00//4
                              ,0x00,0x00,0x00,0x00,0x00//9
                              ,0x00,0x50,0x00,0x50,0x00,0x00//F
                              ,0x02,0x00,0X50,0X00,0X72,0X00//L
                              ,0X50,0X82,0X00,0X00,0X02,0X02//R
                              ,0X00,0X50,0X00,0X28,0X50,0XAA//X
                              ,0X10,0X28//Z
                              ,0x00};//nothing (36)
//unsigned char displayReg [6] = {8,8,8,8,8,8};

void ldIDisplay(unsigned short num,unsigned short bit){ // for individually displaying numbers on the LCD
    if(num>10) num=num-0x37;
    switch (bit){
        case 0: LCDM10 = libL[num];
                LCDM11 = libH[num];break;
        case 1: LCDM6 = libL[num];
                LCDM7 = libH[num];break;
        case 2: LCDM4 = libL[num];
                LCDM5 = libH[num]; break;
        case 3: LCDM19 = libL[num];
                LCDM20 = libH[num]; break;
        case 4: LCDM15 = libL[num];
                LCDM16 = libH[num]; break;
        case 5: LCDM8 = libL[num];
                LCDM9 = libH[num]; break;
        default: break;
    }
    //LCDM16 |= 0x04;// degree sign
    //LCDM7 |= 0x01;// decimal sign
}
void ldADisplayArr(unsigned char arr [],short minDig,short maxDig){// for displaying an already sorted array to LCD
    short x;//initializing x for for-loop



    for(x=minDig;x<maxDig;x++){
        if((int)arr[x] < 0x30){
            ldIDisplay(36,x);
        } else if ((arr[x] <0x3A)){
            ldIDisplay(arr[x]-0x30,x);
        } else if ((arr[x]>0x3A)){
            ldIDisplay(arr[x]-0x37,x);
        }
    }
}
void ldADisplayLng(unsigned long num,short minDig,short maxDig, unsigned short GreatestToLeast){//for displaying a 12 bit number to LCD
    unsigned char arr[12]; // grabs each digit from the number by using modulus
    int countB = maxDig-1;//the digit in the number
    unsigned long temp = num; // for simpler coding, i use a temporary value assigned to num
    while (countB>=0){ // doesn't leave the loop till the number has been counted up to Max Digit
        arr[countB] = temp%10+0x30;
        temp = temp/10;
        countB--;
    }

    ldADisplayArr(arr,minDig,maxDig);// uses a separate array function to display on LCD because of potential future use
}

void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	PM5CTL0 &= ~LOCKLPM5; //Unlock GPIO ports

    P2OUT &=~ (LMOTORF|LMOTORB|RMOTORF|RMOTORB);
    P2DIR |= (LMOTORF|LMOTORB|RMOTORF|RMOTORB);       //Motors initialized as outputs

    //PWM and IR sensor setup
    P3DIR |= PWM;
    P3OUT &=~ PWM;
	P3DIR &=~ (CSENS+LSENS+RSENS);
	P3REN |= (CSENS+LSENS+RSENS);
	P3OUT |= (CSENS+LSENS+RSENS);
	P3IFG =0;
	// LButton settup
	P1DIR &=~ BIT1;
	P1REN |= BIT1;
	P1OUT |= BIT1;
	P1IE |= BIT1;
	P1IES |= BIT1;
	P1IFG = 0;

	//Ultrasound Setup
	P4DIR |= TRIG;
	P4OUT = 0;
	P4DIR &=~ ECHO;
	P4REN |= ECHO;
	P4OUT |= ECHO;
	P4IES &=~ (ECHO + TRIG);
    P4IE |= ECHO + TRIG;
	P4IFG = 0;

	//LCD setup
    LCDCCTL0 &=~ LCDON|LCDSSEL; // turns off LCD
    LCDCCTL0 |= LCDDIV__1|LCDPRE__4|LCD4MUX|LCDLP;
    LCDCPCTL0 = 0b1111111111010000;
    LCDCPCTL1 = 0b1111100000111111;
    LCDCPCTL2 = 0b0000000011111000;
    LCDCMEMCTL |= LCDCLRM; //Clears LCD
    LCDCCTL0 |= LCDON;//turns on LCD

    //Timer for starting TRIG
    TA0CTL |= TASSEL__SMCLK+TACLR;
    TA0CCTL0 |= CCIE;
    //TA0EX0 = 7;
    TA0CCR0 = 60000;

    //timer for measuring ECHO
    TA1CTL = TASSEL__SMCLK +TACLR;
    TA1CCR0 = 65535;

    //PWM
    TA2CTL = TASSEL__SMCLK  + TACLR;;
    TA2CCTL0 |= CCIE;
    TA2CCR0 = 1000;
    //10us timer for Trig
    TA3CTL = TASSEL__SMCLK + TACLR;
    TA3CCTL0 |= CCIE;
    TA3CCR0 = 10;


    __enable_interrupt();
    __no_operation();
    //int scroll =0;

	while(1){}
}
#pragma vector = TIMER3_A0_VECTOR//10us timer for Trig
__interrupt void trigger_timer(void){
    P4OUT &=~ TRIG;
    TA3CTL &=~ MC__UP;
}
#pragma vector = TIMER0_A0_VECTOR //timer to start trig
__interrupt void PULSE(void) {
    P4OUT |= TRIG;

}
#pragma vector = PORT1_VECTOR//turns on other timers when lbtn is pressed, effectively starting the race
__interrupt void MAXIMUMOVERDRIVE(void){
    //P3IE |= CSENS+LSENS+RSENS+PWM;
    TA2CTL |= MC__UP;
    TA0CTL |= MC__UP;
    P1IFG = 0;
}

#pragma vector = PORT3_VECTOR//this is here in case there was extra code that still used the P3 interrupts, preventing an interrupt vector trap
__interrupt void SENSE(void){
    P3IFG=0;
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

    if(P3IN & LSENS){ //SENSOR INPUTS
        lft = 1;
    } else {
        lft = 0;
    }
    if(P3IN & CSENS){
        ctr = 1;
    } else {
        ctr = 0;
    }
    if(P3IN & RSENS){
        rht = 1;
    } else{
        rht = 0;
    }


    int ctrl[3] = {lft,ctr,rht};//used to check for individual sensors

    if(distance <= 13){ //no matter what will stop if ultrasound detects distance of less than 14 cm
        stop();
    } else if(P3IN & PWM){ // PWM for all the driving, stops when high
        stop();
    }
    else if ((!(ctrl[0]))&&(!ctrl[1])&&(!(ctrl[2]))   ){// forward
        straight();
        MEM = P2OUT;
        PWM_COUNTER = 0;
/*                          // remenants of test code to check turning functionality without the hassle of motors running
        ldIDisplay(0,3);
        ldIDisplay(1,4);
        ldIDisplay(0,5);
        //*/
    }else if ((ctrl[0])&&(!ctrl[1])&&(ctrl[2])){// forward
        straight();
        MEM = P2OUT;
        PWM_COUNTER = 0;

        /*
        ldIDisplay(0,3);
        ldIDisplay(1,4);
        ldIDisplay(0,5);
        //*/
    }
    else if (((ctrl[0]))&&(!ctrl[1])&&(!ctrl[2])){//hard left, there was a soft left and right at some point, but was removed do to instability while driving
        hleft();
        MEM = P2OUT;
        PWM_COUNTER = 4;
        /*
        ldIDisplay(0,3);
        ldIDisplay(1,4);
        ldIDisplay(1,5);
        //*/
    }else  if ((!ctrl[0])&&(!ctrl[1])&&((ctrl[2]))){//hard right
        hright();
        MEM = P2OUT;
        PWM_COUNTER = 4;
        /*
        ldIDisplay(0,5);
        ldIDisplay(1,4);
        ldIDisplay(1,3);
        //*/
    }else  if ((!(ctrl[0]))&&((ctrl[1]))&&(ctrl[2])){//hard right
        hright();
        MEM = P2OUT;
        PWM_COUNTER = 4;

        /*
        ldIDisplay(1,5);
        ldIDisplay(0,4);
        ldIDisplay(0,3);
        //*/
    }else  if ((ctrl[0])&&((ctrl[1]))&&(!(ctrl[2]))){//hard left
        hleft();
        MEM = P2OUT;
        PWM_COUNTER = 4;
        /*
        ldIDisplay(1,3);
        ldIDisplay(0,4);
        ldIDisplay(0,5);
        //*/
    }
    else if ((ctrl[0])&&(ctrl[1])&&(ctrl[2])){// if the rover detects all 1's (no black tape) it will remember what its original task was before leaving the tape.
        P2OUT = MEM;//The hopes was that if it leaves the tape while turning, it will bring itself back
    }

}
#pragma vector = PORT4_VECTOR//ECHO AND TRIGGER FUNCTIONS
__interrupt void ECHOTRIGGER(void){
    if(P4IFG & ECHO){//checks both when echo is high and low, uses a seperate timer to count the length of echo.
        TA1CTL |= MC__UP;
        if(P4IES & ECHO) {
            TA1CTL &=~ MC__UP;
            distance = TA1R;
            distance = distance / 58;// Distance in cm;
            ldADisplayLng(distance,0,3,1);
            ldIDisplay('C',4);//hard coded 'CM' on the LCD
            ldIDisplay('M',5);
            TA1CTL |= TACLR;
        }
        P4IES ^= ECHO;
        P4IFG &=~ ECHO;

    }
    if(distance <=13){// if the distance is less than 14 cm, stops the rover.
        stop();
    }
    if(P4IFG & TRIG){ //trig being on for 10us
        P4IFG &=~ TRIG;
        TA3CTL |=MC__UP;
    }
}
