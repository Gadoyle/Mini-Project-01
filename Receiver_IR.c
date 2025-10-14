// rx_ir.c — MSP430FR6989 IR Receiver (TSOP38238 OUT on P1.4)
#include <msp430.h>

#define TSOP_PIN            BIT4      // P1.4 input (active-LOW pulses)
#define IR_THRESH_US_MID    880u      // midpoint between 0 (~0.6–0.76 ms) & 1 (~1.0–1.2 ms)
#define IR_DEAD_LOW_US      850u      // reject ambiguous widths
#define IR_DEAD_HIGH_US     910u
#define IR_BIT_TIMEOUT_US   3000u     // parser reset if no edge > 3 ms
#define IR_BITS_PER_FRAME   4         // demo: 4-bit payload

volatile unsigned ir_low_us = 0;
volatile unsigned ir_bitbuf = 0;
volatile unsigned ir_bitcount = 0;
volatile unsigned ir_frame_ready = 0;
volatile unsigned ir_last_edge_time = 0;

static inline void clocks_init_1MHz(void){
  CSCTL0_H = CSKEY_H;
  CSCTL1   = DCOFSEL_0;                       // ~1 MHz
  CSCTL2   = SELS__DCOCLK | SELM__DCOCLK;
  CSCTL3   = DIVS__1 | DIVM__1;
  CSCTL0_H = 0;
}

static inline void tsop_gpio_init(void){
  PM5CTL0 &= ~LOCKLPM5;
  P1DIR  &= ~TSOP_PIN;
  P1REN  &= ~TSOP_PIN;      // TSOP drives strongly
  P1IES  |=  TSOP_PIN;      // start on falling edge (HIGH->LOW)
  P1IFG  &= ~TSOP_PIN;
  P1IE   |=  TSOP_PIN;
}

static inline void ta0_counter_init(void){
  // TA0 as free-running counter @ ~1 MHz
  TA0CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;
}

static inline void parser_reset(void){
  ir_bitbuf = 0; ir_bitcount = 0; ir_frame_ready = 0;
}

static inline unsigned now_ticks(void){ return TA0R; }
static inline int elapsed_gt(unsigned t0, unsigned limit){ return (unsigned)(now_ticks() - t0) > limit; }

static inline void classify_and_push(unsigned width_us){
  if (width_us > IR_DEAD_LOW_US && width_us < IR_DEAD_HIGH_US){ parser_reset(); return; }
  unsigned bit = (width_us > IR_THRESH_US_MID) ? 1u : 0u;
  ir_bitbuf = (ir_bitbuf << 1) | bit;
  if (++ir_bitcount >= IR_BITS_PER_FRAME) ir_frame_ready = 1;
}

int main(void){
  WDTCTL = WDTPW | WDTHOLD;
  clocks_init_1MHz();
  tsop_gpio_init();
  ta0_counter_init();
  parser_reset();
  ir_last_edge_time = now_ticks();
  __enable_interrupt();

  for(;;){
    if (elapsed_gt(ir_last_edge_time, IR_BIT_TIMEOUT_US)) parser_reset();

    if (ir_frame_ready){
      __bic_SR_register(GIE);
      unsigned payload4 = (ir_bitbuf & 0xF);
      parser_reset();
      __bis_SR_register(GIE);

      // TODO: hook to your rover functions:
      // if (payload4 == 0x1) straight();
      // else if (payload4 == 0x2) right();
      // else cruise();
      (void)payload4;
    }
    __no_operation();
  }
}

// Port 1 ISR — TSOP edges
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void){
  if (P1IFG & TSOP_PIN){
    if (P1IES & TSOP_PIN){
      // Falling edge (HIGH->LOW): burst start → zero counter
      TA0R = 0;
      P1IES &= ~TSOP_PIN;         // next look for rising edge
    } else {
      // Rising edge (LOW->HIGH): burst end → measure width
      ir_low_us = TA0R;           // ~µs at 1 MHz
      classify_and_push(ir_low_us);
      P1IES |= TSOP_PIN;          // next falling edge
    }
    P1IFG &= ~TSOP_PIN;
    ir_last_edge_time = now_ticks();
  }
}
