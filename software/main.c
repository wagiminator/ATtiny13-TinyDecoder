// tinyIRdecoder - IR remote receiver and NEC protocol decoder
//
// ATtiny13 receives IR signal via TSOP4838, decodes the signal
// (NEC protocol) and displays address and command (hex values) on an
// 4-digit 7-segment display via a MAX7219.
//
// For the 4-digit 7-segment display a MAX7219 is controlled via SPI
// protocol with a simple bitbanging method. The MAX7219 is fast enough
// to be driven without delays even at the fastest clock speed of the
// ATtiny13. A transmission to the MAX7219 starts with pulling the CS
// line LOW. Afterwards two bytes are transmitted, the register address
// first and the register data second. The bytes are transmitted most
// significant bit first by setting the DIN line HIGH for a bit "1" or
// LOW for a bit "0" while the CLK line is LOW. The bit is shifted out on
// the rising edge of the CLK line. By setting the CS line HIGH again the
// end of the transmission is signified, the MAX7219 latches the two
// received bytes and writes the data byte into the register. 
//
// The IR NEC decoding function utilizes timer0 to measure the burst and
// pause lengths of the signal. The timer is automatically started and
// stopped or reset by the IR receiver via a pin change interrupt. The
// measured lengths are interpreted according to the NEC protocol and the
// transmitted code is calculated accordingly. The program was tested with
// the TSOP4838, but it should also work with other 38kHz IR receivers
// (note different pinout if necessary).
//
// ------+         +-----+ +-+ +---+ +---+ +-+
//       |         |     | | | |   | |   | | |     bit0:  562.5us
//       |   9ms   |4.5ms| |0| | 1 | | 1 | |0| ...
//       |         |     | | | |   | |   | | |     bit1: 1687.5us
//       +---------+     +-+ +-+   +-+   +-+ +-
//
// The output of the IR reciever is inverted, a burst is indicated by a
// LOW signal, a pause by a HIGH signal. IR message starts with a 9ms
// leading burst followed by a 4.5ms pause. Afterwards 4 data bytes are
// transmitted, least significant bit first. A "0" bit is a 562.5us burst
// followed by a 562.5us pause, a "1" bit is a 562.5us burst followed by
// a 1687.5us pause. A final 562.5us burst signifies the end of the
// transmission. The four data bytes are in order:
// - the 8-bit address for the receiving device,
// - the 8-bit logical inverse of the address,
// - the 8-bit command and
// - the 8-bit logical inverse of the command.
// If the key on the remote controller is kept depressed, a repeat code
// will be issued consisting of a 9ms leading burst, a 2.25ms pause and
// a 562.5us burst to mark the end. The repeat code will continue to be
// sent out at 108ms intervals, until the key is finally released.
// For this application the repeat code will be ignored because it's not
// needed here.
//
//
//             +-------+
// OUT ------ 1|   --  |
// GND ------ 2|  (  ) |  TSOP4838
// VCC ------ 3|   --  |
//             +-------+
//
//                              +-\/-+
//            --- A0 (D5) PB5  1|    |8  Vcc
// MAX7219 CS --- A3 (D3) PB3  2|    |7  PB2 (D2) A1 --- MAX7219 CLK
//            --- A2 (D4) PB4  3|    |6  PB1 (D1) ------ MAX7219 DIN
//                        GND  4|    |5  PB0 (D0) ------ TSOP4838 OUT
//                              +----+    
//
// Controller:  ATtiny13
// Clockspeed:  1.2 MHz internal
//
// 2020 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/


// libraries
#include <avr/io.h>
#include <avr/interrupt.h>

// pin definitions
#define IR_OUT  PB0
#define DIN     PB1
#define CLK     PB2
#define CS      PB3

// definitions for IR receiver
#define IR_WAIT_LOW()   while( PINB & (1<<IR_OUT))  // wait for IR line going LOW
#define IR_WAIT_HIGH()  while(~PINB & (1<<IR_OUT))  // wait for IR line going HIGH
#define IR_8500us       159                         // 8500us * 1.2 MHz / 64
#define IR_4000us       75                          // 4000us * 1.2 MHz / 64
#define IR_1000us       19                          // 1000us * 1.2 MHz / 64

// global variables
volatile uint8_t IR_duration;             // for storing duration of last burst/pause
uint8_t addr, code;                       // for storing address and command code

// shift out byte value to MAX7219
void SEG_byte(uint8_t value) {
  for(uint8_t i=8; i; i--, value <<= 1) { // shift out 8 bits, MSB first
    PORTB &= ~(1<<DIN);                   // clear the bit first (saves some flash)
    if(value & 0x80) PORTB |= (1<<DIN);   // set bit if appropriate
    PORTB |=  (1<<CLK);                   // clock high -> shift out the bit
    PORTB &= ~(1<<CLK);                   // clock low again (50ns < 1000 / 1.2)
  }
}

// send command to MAX7219
void SEG_send(uint8_t reg, uint8_t data) {
  PORTB &= ~(1<<CS);                      // set CS low  -> select device
  SEG_byte(reg);                          // send address byte
  SEG_byte(data);                         // send data byte
  PORTB |= (1<<CS);                       // set CS high -> latch the bytes
}

// initialize MAX7219
void SEG_init(void) {
  DDRB  = (1<<DIN) | (1<<CLK) | (1<<CS);  // set output pins
  PORTB = (1<<CS);                        // CS line HIGH
  SEG_send(0x09, 0x00);                   // no decode
  SEG_send(0x0a, 0x0f);                   // intensity (0x00 .. 0x0f)
  SEG_send(0x0b, 0x03);                   // scan limit 4 digits
  SEG_send(0x0c, 0x01);                   // shutdown mode off
  SEG_send(0x0f, 0x00);                   // display test off
  for(uint8_t i=1; i<5; i++) SEG_send(i, 0x01); // clear all digits
}

// initialize the IR receiver
void IR_init(void) {
  DDRB  &= ~(1<<IR_OUT);                  // IR pin as input
  PCMSK |= (1<<IR_OUT);                   // enable interrupt on IR pin
  TCCR0A = 0;                             // timer/counter normal mode
  TCCR0B = (1<<CS01) | (1<<CS00);         // start the timer, prescaler 64
  sei();                                  // enable global interrupts
}

// read a data byte from IR receiver
uint8_t IR_byte(void) {
  uint8_t databyte;                       // variable for received byte
  for (uint8_t i=8; i; i--) {             // receive 8 bits
    databyte >>= 1;                       // LSB first
    IR_WAIT_HIGH();                       // wait for end of burst
    IR_WAIT_LOW();                        // wait for end of pause
    IR_WAIT_LOW();                        // again for debouncing
    if (IR_duration > IR_1000us) databyte |= 0x80; // bit 0 or 1 depends on pause duration
  }
  return (databyte);                      // return received byte
}

// wait for and read valid IR command (repeat code will be ignored)
void IR_read(void) {
  uint8_t naddr, ncode;                   // variables for inverse data bytes
  GIMSK |= (1<<PCIE);                     // enable pin change interrupts
  do {                                    // loop ...
    IR_WAIT_HIGH();                       // wait for start conditions
    IR_WAIT_LOW();                        // wait for first burst
    IR_WAIT_HIGH();                       // wait for end of first burst
    if (IR_duration < IR_8500us) continue;// if duration < 8.5ms start again
    IR_WAIT_LOW();                        // wait for end of pause
    if (IR_duration < IR_4000us) continue;// if duration < 4 ms start again
    addr  =  IR_byte();                   // read address byte
    naddr = ~IR_byte();                   // read inverse of address byte
    code  =  IR_byte();                   // read command code byte
    ncode = ~IR_byte();                   // read inverse of command code byte
  } while((addr != naddr) || (code != ncode)); // ... until valid code received
  GIMSK &= ~(1<<PCIE);                    // disable pin change interrupts
}

// main function
int main(void) {
  uint8_t SEG_digits[] = {
	  0b01111110, 0b00110000, 0b01101101, 0b01111001,   // 0, 1, 2, 3,
    0b00110011, 0b01011011, 0b01011111, 0b01110000,   // 4, 5, 6, 7,
    0b01111111, 0b01111011, 0b01110111, 0b00011111,   // 8, 9, A, b,
    0b01001110, 0b00111101, 0b01001111, 0b01000111    // C, d, E, F
  };

  SEG_init();                                         // initialize MAX7219
  IR_init();                                          // initialize IR receiver

  while(1) {
    IR_read();                                        // wait for and read IR signal
    SEG_send(0x01, SEG_digits[code &  0x0f]);         // print low nibble of command code
    SEG_send(0x02, SEG_digits[code >> 4]);            // print high nibble of command code
    SEG_send(0x03, SEG_digits[addr &  0x0f] | 0x80);  // print low nibble of address with DP
    SEG_send(0x04, SEG_digits[addr >> 4]);            // print high nibble of address
  }
}

// pin change interrupt service routine
ISR (PCINT0_vect) {
  IR_duration = TCNT0;                    // save timer value
  TCNT0 = 0;                              // reset timer0
}
