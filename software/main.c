// tinyDecoderIR - IR remote receiver and NEC protocol decoder
//
// ATtiny13A receives IR signal via TSOP4838, decodes the signal
// (NEC protocol) and displays address and command (hex values) on an
// SSD1306 128x32 OLED display.
//
// The I²C protocol implementation is based on a crude bitbanging method.
// It was specifically designed for the limited resources of ATtiny10 and
// ATtiny13, but should work with some other AVRs as well.
// The functions for the OLED are adapted to the SSD1306 128x32 OLED module,
// but they can easily be modified to be used for other modules. In order to
// save resources, only the basic functionalities which are needed for this
// application are implemented.
// For a detailed information on the working principle of the I²C OLED
// implementation visit https://github.com/wagiminator/attiny13-tinyoleddemo
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
// The Extended NEC protocol uses 16-bit addresses. Instead of sending
// an 8-bit address and its logically inverse, first the low byte and
// then the high byte of the address is transmitted.
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
//    +-----------------------------+
// ---|SDA +--------------------+   |
// ---|SCL |    SSD1306 OLED    |   |
// ---|VCC |       128x36       |   |
// ---|GND +--------------------+   |
//    +-----------------------------+
//
//                            +-\/-+
//          --- A0 (D5) PB5  1|°   |8  Vcc
// TSOP4838 --- A3 (D3) PB3  2|    |7  PB2 (D2) A1 --- SDA OLED
//          --- A2 (D4) PB4  3|    |6  PB1 (D1) ------ SCL OLED
//                      GND  4|    |5  PB0 (D0) ------
//                            +----+  
//
// Controller:  ATtiny13A
// Clockspeed:  1.2 MHz internal
//
// 2020 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/


// libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// pin definitions
#define I2C_SCL         PB1               // I2C serial clock pin
#define I2C_SDA         PB2               // I2C serial data pin
#define IR_OUT          PB3               // IR receiver pin

// -----------------------------------------------------------------------------
// I2C Implementation
// -----------------------------------------------------------------------------

// I2C macros
#define I2C_SDA_HIGH()  DDRB &= ~(1<<I2C_SDA) // release SDA   -> pulled HIGH by resistor
#define I2C_SDA_LOW()   DDRB |=  (1<<I2C_SDA) // SDA as output -> pulled LOW  by MCU
#define I2C_SCL_HIGH()  DDRB &= ~(1<<I2C_SCL) // release SCL   -> pulled HIGH by resistor
#define I2C_SCL_LOW()   DDRB |=  (1<<I2C_SCL) // SCL as output -> pulled LOW  by MCU

// I2C init function
void I2C_init(void) {
  DDRB  &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // pins as input (HIGH-Z) -> lines released
  PORTB &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // should be LOW when as ouput
}

// I2C transmit one data byte to the slave, ignore ACK bit, no clock stretching allowed
void I2C_write(uint8_t data) {
  for(uint8_t i = 8; i; i--, data<<=1) {  // transmit 8 bits, MSB first
    I2C_SDA_LOW();                        // SDA LOW for now (saves some flash this way)
    if (data & 0x80) I2C_SDA_HIGH();      // SDA HIGH if bit is 1
    I2C_SCL_HIGH();                       // clock HIGH -> slave reads the bit
    I2C_SCL_LOW();                        // clock LOW again
  }
  I2C_SDA_HIGH();                         // release SDA for ACK bit of slave
  I2C_SCL_HIGH();                         // 9th clock pulse is for the ACK bit
  I2C_SCL_LOW();                          // but ACK bit is ignored
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  I2C_SDA_LOW();                          // start condition: SDA goes LOW first
  I2C_SCL_LOW();                          // start condition: SCL goes LOW second
  I2C_write(addr);                        // send slave address
}

// I2C stop transmission
void I2C_stop(void) {
  I2C_SDA_LOW();                          // prepare SDA for LOW to HIGH transition
  I2C_SCL_HIGH();                         // stop condition: SCL goes HIGH first
  I2C_SDA_HIGH();                         // stop condition: SDA goes HIGH second
}

// -----------------------------------------------------------------------------
// OLED Implementation
// -----------------------------------------------------------------------------

// OLED definitions
#define OLED_ADDR       0x78              // OLED write address
#define OLED_CMD_MODE   0x00              // set command mode
#define OLED_DAT_MODE   0x40              // set data mode
#define OLED_INIT_LEN   9                 // 9: no screen flip, 11: screen flip

// OLED init settings
const uint8_t OLED_INIT_CMD[] PROGMEM = {
  0xA8, 0x1F,       // set multiplex (HEIGHT-1): 0x1F for 128x32, 0x3F for 128x64 
  0x20, 0x01,       // set vertical memory addressing mode
  0xDA, 0x02,       // set COM pins hardware configuration to sequential
  0x8D, 0x14,       // enable charge pump
  0xAF,             // switch on OLED
  0xA1, 0xC8        // flip the screen
};

// OLED simple reduced 3x8 font
const uint8_t OLED_FONT[] PROGMEM = {
  0x7F, 0x41, 0x7F, // 0  0
  0x00, 0x00, 0x7F, // 1  1
  0x79, 0x49, 0x4F, // 2  2
  0x41, 0x49, 0x7F, // 3  3
  0x0F, 0x08, 0x7E, // 4  4
  0x4F, 0x49, 0x79, // 5  5
  0x7F, 0x49, 0x79, // 6  6
  0x03, 0x01, 0x7F, // 7  7
  0x7F, 0x49, 0x7F, // 8  8
  0x4F, 0x49, 0x7F, // 9  9
  0x7F, 0x09, 0x7F, // A 10
  0x7F, 0x49, 0x36, // B 11
  0x7F, 0x41, 0x63, // C 12
  0x7F, 0x41, 0x3E, // D 13
  0x7F, 0x49, 0x41, // E 14
  0x7F, 0x09, 0x01, // F 15
  0x41, 0x7F, 0x41, // I 16
  0x7F, 0x02, 0x7F, // M 17
  0x7F, 0x01, 0x7E, // N 18
  0x7F, 0x09, 0x76, // R 19
  0x3F, 0x40, 0x3F, // V 20
  0x00, 0x36, 0x00, // : 21
  0x00, 0x00, 0x00  //   22
};

// OLED init function
void OLED_init(void) {
  I2C_init();                             // initialize I2C first
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  for (uint8_t i = 0; i < OLED_INIT_LEN; i++) 
    I2C_write(pgm_read_byte(&OLED_INIT_CMD[i])); // send the command bytes
  I2C_stop();                             // stop transmission
}

// OLED set the cursor
void OLED_setCursor(uint8_t xpos, uint8_t ypos) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  I2C_write(0x22);                        // command for min/max page
  I2C_write(ypos); I2C_write(ypos+1);     // min: ypos; max: ypos+1
  I2C_write(xpos & 0x0F);                 // set low nibble of start column
  I2C_write(0x10 | (xpos >> 4));          // set high nibble of start column
  I2C_write(0xB0 | (ypos));               // set start page
  I2C_stop();                             // stop transmission
}

// OLED clear screen
void OLED_clearScreen(void) {
  OLED_setCursor(0, 0);                   // set cursor at upper half
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  uint8_t i = 0;                          // count variable
  do {I2C_write(0x00);} while (--i);      // clear upper half
  I2C_stop();                             // stop transmission
  OLED_setCursor(0, 2);                   // set cursor at lower half
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  do {I2C_write(0x00);} while (--i);      // clear upper half
  I2C_stop();                             // stop transmission
}

// OLED stretch 8-bit value (x) to 16-bit and write it several times (t)
// abcdefgh -> aabbccddeeffgghh
// refer to http://www.technoblogy.com/show?LKP
void OLED_stretch (uint16_t x, uint8_t t) {
  x  = (x & 0xF0)<<4 | (x & 0x0F);
  x  = (x<<2 | x) & 0x3333;
  x  = (x<<1 | x) & 0x5555;
  x |= x<<1;
  for (; t; t--) {                        // print t-times on the OLED
    I2C_write(x);                         // write low  byte
    I2C_write(x>>8);                      // write high byte
  }
}

// OLED print a big character
void OLED_printChar(uint8_t ch) {
  ch += ch << 1;                          // calculate position of character in font array
  OLED_stretch(pgm_read_byte(&OLED_FONT[ch++]), 2);
  OLED_stretch(pgm_read_byte(&OLED_FONT[ch++]), 3);
  OLED_stretch(pgm_read_byte(&OLED_FONT[ch  ]), 2);
  for(ch=4; ch; ch--) I2C_write(0x00);    // print spacing between characters
}

// OLED print a string from program memory; terminator: 255
void OLED_printString(const uint8_t* p) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  uint8_t ch = pgm_read_byte(p);          // read first character from program memory
  while (ch < 255) {                      // repeat until string terminator
    OLED_printChar(ch);                   // print character on OLED
    ch = pgm_read_byte(++p);              // read next character
  }
  I2C_stop();                             // stop transmission
}

// OLED print byte as hex value
void OLED_printHex(uint8_t val) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  OLED_printChar(val>>4);                 // print high nibble of the byte
  OLED_printChar(val & 0x0F);             // print low  nibble of the byte
  I2C_stop();                             // stop transmission
}

// -----------------------------------------------------------------------------
// IR Receiver Implementation
// -----------------------------------------------------------------------------

// IR receiver definitions and macros
#define IR_WAIT_LOW()   while( PINB & (1<<IR_OUT))  // wait for IR line going LOW
#define IR_WAIT_HIGH()  while(~PINB & (1<<IR_OUT))  // wait for IR line going HIGH
#define IR_9000us       169                         // 9000us * 1.2 MHz / 64
#define IR_4500us       84                          // 4500us * 1.2 MHz / 64
#define IR_1687us       32                          // 1687us * 1.2 MHz / 64
#define IR_562us        11                          //  562us * 1.2 MHz / 64
#define IR_FAIL         0
#define IR_NEC          1

// IR global variables
volatile uint8_t IR_duration;             // for storing duration of last burst/pause
uint16_t addr;                            // for storing address code
uint8_t  cmd;                             // for storing command code

// IR initialize the receiver
void IR_init(void) {
  DDRB  &= ~(1<<IR_OUT);                  // IR pin as input
  PCMSK |= (1<<IR_OUT);                   // enable interrupt on IR pin
  TCCR0A = 0;                             // timer/counter normal mode
  TCCR0B = (1<<CS01) | (1<<CS00);         // start the timer, prescaler 64
  sei();                                  // enable global interrupts
}

// IR check if current signal length matches the desired duration
uint8_t IR_checkDur(uint8_t dur) {
  uint8_t error = dur >> 3; if (error < 6) error = 6;
  if (IR_duration > dur) return ((IR_duration - dur) < error);
  return ((dur - IR_duration) < error);
}

// IR read data according to NEC protocol
uint8_t IR_readNEC(void) {
  uint32_t data;
  IR_WAIT_LOW();                          // wait for end of start pause
  if (!IR_checkDur(IR_4500us)) return 0;  // exit if no start condition
  for (uint8_t i=32; i; i--) {            // receive 32 bits
    data >>= 1;                           // LSB first
    IR_WAIT_HIGH();                       // wait for end of burst
    if (!IR_checkDur(IR_562us)) return 0; // exit if burst has incorrect length
    IR_WAIT_LOW();                        // wait for end of pause
    if (IR_checkDur(IR_1687us)) data |= 0x80000000; // bit "0" or "1" depends on pause duration
    else if (!IR_checkDur(IR_562us)) return 0;      // exit if it's neither "0" nor "1"
  }
  IR_WAIT_HIGH();                         // wait for end of final burst
  if (!IR_checkDur(IR_562us)) return 0;   // exit if burst has incorrect length
  uint8_t addr1 = data;                   // get first  address byte
  uint8_t addr2 = data >> 8;              // get second address byte
  uint8_t cmd1  = data >> 16;             // get first  command byte
  uint8_t cmd2  = data >> 24;             // get second command byte
  if ((cmd1 + cmd2) < 255) return 0;      // if second command byte is not the inverse of the first
  cmd = cmd1;                             // get the command
  if ((addr1 + addr2) == 255) addr = addr1;   // check if it's extended NEC-protocol ...
  else addr = data;                       // ... and get the correct address
  return IR_NEC;                          // return NEC success
}

// IR wait for and read valid IR command (repeat code will be ignored)
uint8_t IR_read(void) {
  uint8_t protocol = IR_FAIL;             // variables for received protocol
  GIMSK |= (1<<PCIE);                     // enable pin change interrupts
  do {                                    // loop ...
    IR_WAIT_HIGH();                       // wait for start conditions
    IR_WAIT_LOW();                        // wait for first burst
    IR_WAIT_HIGH();                       // wait for end of first burst
    if (IR_checkDur(IR_9000us))           // if NEC start condition
      protocol = IR_readNEC();            //   read NEC
  } while(!protocol);                     // ... until valid code received
  GIMSK &= ~(1<<PCIE);                    // disable pin change interrupts
  return protocol;
}

// pin change interrupt service routine
ISR (PCINT0_vect) {
  IR_duration = TCNT0;                    // save timer value
  TCNT0 = 0;                              // reset timer0
}

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

// some "strings"
const uint8_t ADR[] PROGMEM = {10, 13, 13, 19, 14,  5,  5, 21, 22, 22, 255};
const uint8_t CMD[] PROGMEM = {12,  0, 17, 17, 10, 18, 13, 21, 22, 22, 22, 22, 255};
const uint8_t IRR[] PROGMEM = { 1, 19, 22, 18, 14, 12, 22, 13, 14, 12,  0, 13, 14, 19, 255};
const uint8_t SPC[] PROGMEM = {22, 22, 255};

// main function
int main(void) {
  // setup
  IR_init();                              // initialize IR receiver
  OLED_init();                            // initialize the OLED
  OLED_clearScreen();                     // clear screen
  OLED_setCursor(0,1);                    // set cursor to start of second line
  OLED_printString(IRR);                  // print "IR NEC DECODER"

  // loop
  while(1) {                              // loop until forever
    IR_read();                            // wait for and read IR signal
    OLED_setCursor(0,0);                  // set cursor to start of first line
    OLED_printString(ADR);                // print "ADDRESS: "
    if (addr > 255) OLED_printHex(addr >> 8);  // extended NEC
    else OLED_printString(SPC);
    OLED_printHex(addr);                  // print received address
    OLED_setCursor(0,2);                  // set cursor to start of third line
    OLED_printString(CMD);                // print "COMMAND: "
    OLED_printHex(cmd);                   // print received command
  }
}
