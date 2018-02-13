# MODBUS data documentation



> Sensors data management (mainapp MODE)



  **Holding Registers (0x03)**

  | Addr (regs)     | Length (type)     |                          Action                            |
  | --------------- | ----------------- | ---------------------------------------------------------- |
  | 0x0000 (1 regs) | 2 byte (uint16_t) | Running mode (0/mainapp 1/bootload)                        |
  | 0x0001 (2 regs) | 4 byte (char[4])  | Software version string (ASCII)                            |
  | 0x0002 (1 regs) | 2 byte (uint16_t) | Slave address (0x01-0xfe valid range)                      |
  | 0x0003 (2 regs) | 4 byte (float)    | Internal Vcc voltage (Volts)                               |
  | 0x0004 (2 regs) | 4 byte (float)    | Internal temperature (Celsius)                             |

  **Input Registers (0x04)**

  | Addr (regs)     | Length (type)     |                          Action                            |
  | --------------- | ----------------- | ---------------------------------------------------------- |
  | 0x0000 (1 regs) | 2 byte (uint16_t) | 1WIRE: Maximum amount of devices permitted                 |
  | 0x0001 (1 regs) | 2 byte (uint16_t) | 1WIRE: Search and return amount of devices found           |
  |                 |                   |   **0x11 Failed to Respond (BUS error)**                   |
  | 0x01xx (4 regs) | 8 byte (uint64_t) | 1WIRE: 64bit unique dID of device XX in table              |
  |                 |                   |   **0x02 Illegal data address (device XX does not exist)** |
  | 0x02xx (2 regs) | 4 byte (float)    | 1WIRE: Read the temperature from device XX (Celsius)       |
  |                 |                   |   **0x02 Illegal data address (device XX does not exist)** |
  |                 |                   |   **0x08 Parity Error         (device XX CRC failed)**     |
  |                 |                   |   **0x11 Failed to Respond    (device XX BUS failed)**     |
  | 0x1200 (2 regs) | 4 byte (float)    | I2C: Read temperture from SHT21 device (Celsius)           |
  | 0x1201 (2 regs) | 4 byte (float)    | I2C: Read R humidity from SHT21 device (Percent)           |
  | 0x1210 (2 regs) | 4 byte (float)    | I2C: Read visible light lux from SI1145 device (Lux)       |
  | 0x1211 (2 regs) | 4 byte (float)    | I2C: Read ifrared light lux from SI1145 device (Lux)       |
  | 0x1211 (2 regs) | 4 byte (float)    | I2C: Read ultraviolet index from SI1145 device (Index)     |
  | 0x1230 (2 regs) | 4 byte (float)    | I2C: Read temperature from BMP280 device (Celsius)         |
  | 0x1231 (2 regs) | 4 byte (float)    | I2C: Read air pressure from BMP280 device (hPa)            |

  **Write Registers (0x06)**

  | Addr (regs)     | Length (type)     |                          Action                            |
  | --------------- | ----------------- | ---------------------------------------------------------- |
  | 0x0000 (1 regs) | 2 byte (uint16_t) | Change running MODE (0/mainapp 1/bootload)                 |
  |                 |                   |   **0x03 Invalid data value (must be 0x01)**               |
  | 0x0001 (1 regs) | 2 byte (uint16_t) | Change slave address                                       |
  |                 |                   |   **0x03 Invalid data value (must be in 0x01-0xfe range)** |



> Firmware upgrade (bootload MODE)



  **Write Multiple Registers (0x10)**

  | Addr (regs)     | Length (type)     |                          Action                            |
  | --------------- | ----------------- | ---------------------------------------------------------- |
  | 0xXXXX (16 regs)| 32 byte (uint16_t)| Burn data to flash 0x0000-0x1C000 (32 byte at once)        |
  |                 |                   |  **0x02 Illegal data address (only 0x0000-0x1C000, x32)**  |

  **Holding Registers (0x03)**

  | Addr (regs)     | Length (type)     |                          Action                            |
  | --------------- | ----------------- | ---------------------------------------------------------- |
  | 0x0000 (1 regs) | 2 byte (uint16_t) | Running mode (0/mainapp 1/bootload)                        |
  | 0x0001 (2 regs) | 4 byte (char[4])  | Software version string (ASCII)                            |

  **Write Registers (0x06)**

  | Addr (regs)     | Length (type)     |                          Action                            |
  | --------------- | ----------------- | ---------------------------------------------------------- |
  | 0x0000 (1 regs) | 2 byte (uint16_t) |  Change running MODE (0/mainapp 1/bootload)                |
  |                 |                   |    **0x03 Invalid data value  (must be 0x00)**             |
