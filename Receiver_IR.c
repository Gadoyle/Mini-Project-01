// rx_uart_ir.c — MSP430FR6989 IR Receiver over UART (TSOP32238 → eUSCI_Ax RXD)
#include <msp430.h>
#include <stdint.h>

#define USE_EUSCI_A0              1   // set to 0 if you switch to A1/A2

// Baud: 9600, 8N1, SMCLK = 16 MHz (easy, reliable combo)
#define UART_BAUD_9600

static void clock_init_16mhz(void) {
  // DCO = 16 MHz, SMCLK = 16 MHz
  CSCTL0_H = CSKEY_H;
  CSCTL1   = DCOFSEL_4;                    // 16 MHz DCO
  CSCTL2   = SELS__DCOCLK | SELM__DCOCLK | SELA__VLOCLK;
  CSCTL3   = DIVS__1     | DIVM__1     | DIVA__1;
  CSCTL0_H = 0;
}

static void uart_pins_init(void) {
  PM5CTL0 &= ~LOCKLPM5;                    // unlock GPIO (FRAM devices)

#if USE_EUSCI_A0

  P2DIR  &= ~BIT1;                         // P2.1 as input (RXD)
  P2SEL0 |=  BIT1;
  P2SEL1 &= ~BIT1;

  // If you also wire TX for debugging (optional):
  // P2DIR  |=  BIT0;                      // P2.0 as output (TXD)
  // P2SEL0 |=  BIT0;
  // P2SEL1 &= ~BIT0;

#else
  // eUSCI_A1 example
  // Example: P3.4 = UCA1RXD
  P3DIR  &= ~BIT4;
  P3SEL0 |=  BIT4;
  P3SEL1 &= ~BIT4;
#endif
}

static void uart_init_9600_SMCLK16MHz(void) {
#if USE_EUSCI_A0
  UCA0CTLW0 = UCSWRST;                     // hold USCI in reset
  UCA0CTLW0 |= UCSSEL__SMCLK;              // SMCLK as BRCLK (16 MHz)

  // 9600 bps @ 16 MHz with oversampling:
  // N = 16,000,000 / 9600 ≈ 1666.6667
  // BRW = 104, UCBRF = 2, UCOS16 = 1, UCBRS ≈ 0 (good enough for lab use)
  UCA0BRW   = 104;
  UCA0MCTLW = UCOS16 | UCBRF_2 | (0 << 8); // UCBRSx = 0

  UCA0CTLW0 &= ~UCSWRST;                   // release reset
  UCA0IE    |= UCRXIE;                     // enable RX interrupt
#else
  UCA1CTLW0 = UCSWRST;
  UCA1CTLW0 |= UCSSEL__SMCLK;

  UCA1BRW   = 104;
  UCA1MCTLW = UCOS16 | UCBRF_2 | (0 << 8);

  UCA1CTLW0 &= ~UCSWRST;
  UCA1IE    |= UCRXIE;
#endif
}

//Command dispatcher (maps 4-bit payload to actions)
static inline void handle_cmd_4bit(uint8_t cmd4) {
  switch (cmd4 & 0x0F) {
    case 0x1: /* straight();   */ break; // 0001
    case 0x2: /* right();      */ break; // 0010
    case 0x4: /* left();       */ break; // 0100
    case 0x8: /* backwards();  */ break; // 1000
    case 0x9: /* armUp();      */ break; // 1001
    case 0x6: /* armDown();    */ break; // 0110
    default:  /* ignore / NOP  */ break;
  }
}

int main(void) {
  WDTCTL = WDTPW | WDTHOLD;

  clock_init_16mhz();
  uart_pins_init();
  uart_init_9600_SMCLK16MHz();

  __enable_interrupt();

  for (;;) {
    __no_operation();
  }
}

// UART RX ISR 

#if USE_EUSCI_A0
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
  if (UCA0IFG & UCRXIFG) {
    uint8_t b = UCA0RXBUF;     // received byte from TSOP32238 → UART RXD
    handle_cmd_4bit(b);        // use low nibble as your 4-bit command
  }
}
#else
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
  if (UCA1IFG & UCRXIFG) {
    uint8_t b = UCA1RXBUF;
    handle_cmd_4bit(b);
  }
}
#endif
