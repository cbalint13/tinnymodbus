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


# create connection (boot mode is 9600)
client = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=9600, timeout=1.5)
client.connect()


idslave = 0x01

# get running mode
print "modbus cmd: 0x03 value: 0x0000 length: 0x01\n"
result  = client.read_holding_registers(address=0x0000, count=0x01, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
print decoder.decode_16bit_int(), " (running mode)\n",

print

# get loader version
print "modbus cmd: 0x03 value: 0x0001 length: 0x02\n"
result  = client.read_holding_registers(address=0x0001, count=0x02, unit=idslave)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Big)
x = decoder.decode_32bit_int();
print ''.join(chr((x>>8*(4-byte-1))&0xFF) for byte in range(4)) , " (software version)\n",

print

client.close()
