# TinyDecoder - IR Remote Receiver and NEC Decoder based on ATtiny13
ATtiny13 receives IR signal via TSOP4838, decodes the signal (NEC protocol) and displays address and command (hex values) on a 4-digit 7-segment display via a MAX7219.

For the 4-digit 7-segment display a MAX7219 is controlled via SPI protocol with a simple bitbanging method. The MAX7219 is fast enough to be driven without delays even at the fastest clock speed of the ATtiny13. A transmission to the MAX7219 starts with pulling the CS line LOW. Afterwards two bytes are transmitted, the register address first and the register data second. The bytes are transmitted most significant bit first by setting the DIN line HIGH for a bit "1" or LOW for a bit "0" while the CLK line is LOW. The bit is shifted out on the rising edge of the CLK line. By setting the CS line HIGH again the end of the transmission is signified, the MAX7219 latches the two received bytes and writes the data byte into the register. 

The IR NEC decoding function utilizes timer0 to measure the burst and pause lengths of the signal. The timer is automatically started and stopped or reset by the IR receiver via a pin change interrupt. The measured lengths are interpreted according to the NEC protocol and the transmitted code is calculated accordingly. The program was tested with the TSOP4838, but it should also work with other 38kHz IR receivers (note different pinout if necessary).

The output of the IR reciever is inverted, a burst is indicated by a LOW signal, a pause by a HIGH signal. IR message starts with a 9ms leading burst followed by a 4.5ms pause. Afterwards 4 data bytes are transmitted, least significant bit first. A "0" bit is a 562.5us burst followed by a 562.5us pause, a "1" bit is a 562.5us burst followed by a 1687.5us pause. A final 562.5us burst signifies the end of the transmission. The four data bytes are in order:
- the 8-bit address for the receiving device,
- the 8-bit logical inverse of the address,
- the 8-bit command and
- the 8-bit logical inverse of the command.

If the key on the remote controller is kept depressed, a repeat code will be issued consisting of a 9ms leading burst, a 2.25ms pause and a 562.5us burst to mark the end. The repeat code will continue to be sent out at 108ms intervals, until the key is finally released. For this application the repeat code will be ignored because it's not needed here.
