#!/usr/bin/env python
from PybSerial import *

def test1():
    s = Serial(port='4', baudrate=9600)
    a = s.write(bytes([2,3,4]))
    print(a)
    a = s.read()
    print(a)

if __name__ == '__main__':
    test1()
