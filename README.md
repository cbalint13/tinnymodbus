# TINNYMODBUS (ModBus tinny multi-sensor module)

TinnyModBus is a very small 11x26mm reconfigurable atmel tinny85 mcu based micro-module that speaks modbus over rs485 wires.

![Logo](https://github.com/cbalint13/tinnymodbus/raw/master/docs/tinnymodbus-pcb.png)

> Main specs

  - Size is only 11 x 26 mm including screw sockets
  - Can speak modbus with several sensors attached, uses a max487 to communicate
  - Have it's own 500mA budget, ESD protected rs485 lines, and reverse-polarity protection
  - Can read attached sensor data over 1wire, i2c, spi, but olso could do gpio or adc
  - Has its own bootloader that speaks modbus and can be reprogrammed right on rs485 wires
  - Internal IC metrics like Vcc voltage and SOIC8 temperature are available
  - It is designed to cost less than 2 USD

Schematic, Printed Circuit Board, 3D layout will be released on CircuitMaker.com, but can be also checked in **docs** folder.


More docs & usage scenario examples to come.


![SCH](https://github.com/cbalint13/tinnymodbus/raw/master/docs/tinnymodbus-sch.png)
![3D](https://github.com/cbalint13/tinnymodbus/raw/master/docs/tinnymodbus-3d.png)

