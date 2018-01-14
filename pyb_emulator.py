# -*- coding: utf-8 -*-
#===========================================================
# pyb emulator
#
# Designed by Aixi Wang  (aixi.wang@hotmail.com)
#===========================================================

import serial

def unique_id():
    return b'\x40\x00\x37\x00\x16\x47\x33\x31\x38\x33\x36\x32'

class LED:
    def __init__(self,i):
        self.i = i
    
    def on(self):      
        print('led ' + str(self.i) + ' on')
        
    def off(self):
        print('led ' + str(self.i) + ' off')

    def swap(self):
        print('led ' + str(self.i) + ' swap')

class UART:
    def __init__(self,uart_port,uart_baud):
        if uart_port == '4':
            print('pyboard port 4 is initialized')
            self.uart_port = 'COM13'
            self.uart_baud = uart_baud
            self.ser = serial.Serial(self.uart_port, self.uart_baud, timeout=1)
        else:
            print('invalid uart port')
            self.ser = None

    def write(self,s):
        #print 'write, hex:',s,' len:',len(s)
        self.ser.write(s)

    def read(self):
        s = self.ser.read()
        #print 'read, hex:',s.encode('hex')
        return s
        
    def read(self,n):
        s = self.ser.read(n)
        #print 'read, hex:',s.encode('hex')
        return s
        
    def close(self):
        s = self.ser.close()
        return s
        