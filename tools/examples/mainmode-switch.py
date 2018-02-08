#!/usr/bin/python


import sys
import logging

from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.client.sync import ModbusSerialClient as ModbusClient


logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.INFO)


# create connection (main mode is 38400)
client = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=38400, timeout=1.5)
client.connect()


idslave = 0x01

print "modbus cmd: 0x01 value: 0x0001 length: 0x01\n"
result  = client.write_register(address=0x0000, value=0x0001, count=0x01, unit=idslave)
print result

print

client.close()

