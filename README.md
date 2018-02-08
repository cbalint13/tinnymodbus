# TINNYMODBUS (ModBus tinny multi-sensor module)

TinnyModBus is a very small 11x26mm reconfonfigurable atinny85 mcu based micro module that speaks modbus over remote rs485 wires.

> Main specs

  - Size is only 11 x 26 mm including screw sockets
  - Can speak modbus with several sensors attached, uses a max487 to communicate
  - Have it's own 500mA budget, ESD protected rs485 lines, and reverse-polarity protection
  - Can read attached sensor data over 1wire, i2c, spi, but olso could do gpio or adc
  - Has its own bootloader that speaks modbus and can be reprogrammed right on rs485 wires
  - Internal IC metrics like Vcc voltage and SOIC8 temperature are available
  - It is designed to cost less than 2 USD

> Scehmatics and Printed Circuit Boards, 3D layout will be released on CircuitMaker.com, but can be checked in **docs** folder.
