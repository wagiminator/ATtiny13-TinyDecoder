// tinyDecoderIR - IR remote receiver and protocol decoder
//
// ATtiny25 receives IR signal via TSOP4838, decodes the signal and
// displays address and command (hex values) on an SSD1306 128x32 OLED
// display.
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
// The IR decoding function is taken from David Johnson-Davies' IR Remote
// Control Detective: http://www.technoblogy.com/show?24A9
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
// Controller:  ATtiny25
// Clockspeed:  1 MHz internal
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

// simple reduced 3x8 font
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
  0x4F, 0x48, 0x7F, // Y 16
  0x7F, 0x02, 0x7F, // M 17
  0x7F, 0x01, 0x7E, // N 18
  0x7F, 0x09, 0x76, // R 19
  0x08, 0x08, 0x08, // - 20
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

// OLED stretch 8-bit (x) to 16-bit and write it several times (t)
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

// OLED plot a big character
void OLED_plotChar(uint8_t ch) {
  ch += ch << 1;                          // calculate position of character in font array
  OLED_stretch(pgm_read_byte(&OLED_FONT[ch++]), 2);
  OLED_stretch(pgm_read_byte(&OLED_FONT[ch++]), 3);
  OLED_stretch(pgm_read_byte(&OLED_FONT[ch  ]), 2);
  for(ch=4; ch; ch--) I2C_write(0x00);    // print spacing between characters
}

// OLED print a character on a specified position
void OLED_printChar(uint8_t ch, uint8_t xpos, uint8_t ypos) {
  OLED_setCursor(xpos, ypos);             // set cursor
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  OLED_plotChar(ch);                      // plot the character
  I2C_stop();                             // stop transmission
}

// OLED print a string from program memory; terminator: 255
void OLED_printString(const uint8_t* p, uint8_t xpos, uint8_t ypos) {
  OLED_setCursor(xpos, ypos);             // set cursor
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  uint8_t ch = pgm_read_byte(p);          // read first character from program memory
  while (ch < 255) {                      // repeat until string terminator
    OLED_plotChar(ch);                    // plot character on OLED
    ch = pgm_read_byte(++p);              // read next character
  }
  I2C_stop();                             // stop transmission
}

// OLED print 16-bit value as hex
void OLED_printHex(uint16_t val, uint8_t xpos, uint8_t ypos) {
  OLED_setCursor(xpos, ypos);             // set cursor
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  OLED_plotChar((val>>12) & 0x0F);        // print high nibble of high byte
  OLED_plotChar((val>>8)  & 0x0F);        // print low  nibble of high byte
  OLED_plotChar((val>>4)  & 0x0F);        // print high nibble of low  byte
  OLED_plotChar(val       & 0x0F);        // print low  nibble of low  byte
  I2C_stop();                             // stop transmission
}

// -----------------------------------------------------------------------------
// IR Receiver Implementation (from http://www.technoblogy.com/show?24A9)
// -----------------------------------------------------------------------------

// global variables
volatile uint32_t RecdData;
volatile int8_t   Bit, Edge;
volatile uint8_t  IRtype;
uint8_t StartScreen = 0;
  
// protocol timings
const uint8_t Micros =        64;   // number of microseconds per TCNT1 count
const uint8_t SonyIntro =     37;   // 2400/Micros
const uint8_t SonyMean =      14;   //  900/Micros
const uint8_t NECIntro =     140;   // 9000/Micros
const uint8_t NECGap =        70;   // 4500/Micros
const uint8_t NECPulse =       8;   //  562/Micros
const uint8_t NECMean =       17;   // 1125/Micros
const uint8_t SamsungIntro =  78;   // 5000/Micros
const uint8_t SamsungGap =    78;   // 5000/Micros
const uint8_t RC5Half =       13;   //  889/Micros
const uint8_t RC5Full =       27;   // 1778/Micros
const uint8_t RC5Mean =       20;   // 1334/Micros

// IR types
const uint8_t NECtype = 1;
const uint8_t SAMtype = 2;
const uint8_t RC5type = 3;
const uint8_t SONtype = 4;

// some "strings"
const uint8_t IRD[] PROGMEM = { 1, 19, 22, 13, 14, 12, 0, 13, 14, 19, 255};  // "IR DECODER"
const uint8_t ADR[] PROGMEM = {10, 13, 19, 21, 255};  // "ADR:"
const uint8_t CMD[] PROGMEM = {12, 17, 13, 21, 255};  // "CMD:"
const uint8_t IRT[] PROGMEM = {22, 22, 22, 22, 255,   // "    "
                               18, 14, 12, 22, 255,   // "NEC "
                                5, 10, 17,  5, 255,   // "SAMS"
                               19, 12, 20,  5, 255,   // "RC-5"
                                5,  0, 18, 16, 255};  // "SONY"

// print received code
void ReceivedCode(uint8_t IRtype, uint16_t Address, uint16_t Command) {
  if (!StartScreen) {
    StartScreen = 1;
    OLED_clearScreen();
    OLED_printString(ADR, 50, 0);
    OLED_printString(CMD, 50, 2);
  }
  if (IRtype < 4) {
    OLED_printString(IRT + IRtype + (IRtype<<2), 0, 0);
    OLED_printChar(22, 9, 2); OLED_printChar(22, 18, 2);
  }
  else {
     OLED_printString(IRT + 20, 0, 0);
     OLED_printChar(IRtype/10, 9, 2); OLED_printChar(IRtype%10, 18, 2);
  }
  OLED_printHex(Address, 92, 0);
  OLED_printHex(Command, 92, 2);
}

// calculate difference
uint16_t diff(uint16_t a, uint16_t b) {
  if (a > b) return (a - b);
  return (b - a);
}

// interrupt service routine - called on Timer/Counter1 overflow
ISR(TIMER1_OVF_vect) {
  ReceivedCode(Bit, RecdData>>7, RecdData & 0x7F);
  TIMSK = TIMSK & ~(1<<TOIE1);                  // disable overflow interrupt
  TCNT1 = 250;
}

// interrupt service routine - called on each edge of IR pin
ISR(PCINT0_vect) {
  static uint8_t Mid;                            // edge: 0 = falling, 1 = rising
  uint16_t Time = TCNT1;
  if (TIFR & 1<<TOV1) { IRtype = 0; Edge = 1; } // overflow
  else if (Edge != (PINB>>PINB3 & 1));          // ignore if wrong edge
  else if (IRtype == 0) {
  
    // end of intro pulse
    if (diff(Time, RC5Half) < 5) {
      IRtype = RC5type; RecdData = 0x2000; Bit = 12; Edge = 0; Mid = 0;
    } else if (diff(Time, RC5Full) < 5) {
      IRtype = RC5type; RecdData = 0x2000; Bit = 11; Edge = 0; Mid = 1;
    } else if ((diff(Time, SonyIntro) < 5) && (Edge == 1)) {
      IRtype = SONtype; RecdData = 0; Bit = 0;
      TIMSK = TIMSK | 1<<TOIE1;                            // enable overflow interrupt
    } else if (diff(Time, SamsungIntro) < 18) {
      IRtype = SAMtype; RecdData = 0; Bit = -1; Edge = 0;  // ignore first falling edge
    } else if (diff(Time, NECIntro) < 18) { 
      IRtype = NECtype; RecdData = 0; Bit = -1; Edge = 0;  // ignore first falling edge
    }
  
  // data bit
  } else if (IRtype == RC5type) {
    Edge = !Edge;
    if ((Time < RC5Mean) && Mid) {
      Mid = 0;
    } else {
      Mid = 1;
      RecdData = RecdData | ((uint32_t) Edge<<Bit);
      if (Bit == 0) ReceivedCode(RC5type, RecdData>>6 & 0x1F,
        (~(RecdData>>6) & 0x40) | (RecdData & 0x3F));
      Bit--;
    }
  } else if (IRtype == SONtype) {
    if (Time > SonyMean) RecdData = RecdData | ((uint32_t) 1<<Bit);
    Bit++;
  } else if ((IRtype == NECtype) || (IRtype == SAMtype)) {
    if (Time > NECMean && Bit >= 0) RecdData = RecdData | ((uint32_t) 1<<Bit);
    Bit++;
    if (Bit == 32) ReceivedCode(IRtype, RecdData & 0xFFFF, RecdData>>16);
  }
  
  TCNT1 = 0;                  // clear counter
  TIFR = TIFR | 1<<TOV1;      // clear overflow
}

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

int main(void) {
  // setup
  OLED_init();
  OLED_clearScreen();
  TCCR1 = 7<<CS10;              // no compare matches ; /64
  PCMSK = 1<<IR_OUT;            // interrupt on IR pin
  GIMSK = 1<<PCIE;              // enable pin change interrupts
  sei();                        // enable global interrupts

  // print heading
  OLED_printString(IRD, 18, 1);

  // loop
  while(1);                     // everything done under interrupt!
}
