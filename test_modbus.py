#!/usr/bin/env python
from minimalmodbus import *

def test1():
    modbus = PybModbus('4', 1)
    a = modbus.read_register(0, 1)
    print('register 0, length 1, data:' + str(a))

if __name__ == '__main__':
    test1()
