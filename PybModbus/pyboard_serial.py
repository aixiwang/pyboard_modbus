#!/usr/bin/env python
#
#   Copyright 2012 Jonas Berg
#   Copyright 2016 Aixi Wang

#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#

__author__  = 'Jonas Berg'
__email__   = 'pyhys@users.sourceforge.net'
__license__ = 'Apache License, Version 2.0'

import sys
import time

try:
    import pyb
#from pyb import UART
except:
    import pyb_emualtor as pyb
    


PARITY_NONE =  None
PARITY_ODD = 1
PARITY_EVEN = 0

DEFAULT_TIMEOUT = 5
DEFAULT_BAUDRATE = 9600
VERBOSE = True
RESPONSES = {}
RESPONSES['EXAMPLEREQUEST'] = 'EXAMPLERESPONSE'
DEFAULT_RESPONSE = 'NONE'
NO_DATA_PRESENT = ''

class Serial():
    def __init__(self, *args, **kwargs):
        self._waiting_data = NO_DATA_PRESENT
        self._isOpen = True
        self.port = kwargs['port']  # Serial port name.
        self.initial_port_name = self.port  # Initial name given to the serial port
        try:
            self.timeout = kwargs['timeout']
        except:
            self.timeout = DEFAULT_TIMEOUT
        try:
            self.baudrate = kwargs['baudrate']
        except:
            self.baudrate = DEFAULT_BAUDRATE

        if VERBOSE:
            _print_out('\nDummy_serial: Initializing')
            _print_out('dummy_serial initialization args: ' + repr(args) )
            _print_out('dummy_serial initialization kwargs: ' + repr(kwargs) + '\n')
            
        self.ser = pyb.UART(self.port,self.baudrate)

    def __repr__(self):
        """String representation of the dummy_serial object"""
        return "{0}.{1}<id=0x{2:x}, open={3}>(port={4!r}, timeout={5!r}, waiting_data={6!r})".format(
            self.__module__,
            self.__class__.__name__,
            id(self),
            self._isOpen,
            self.port,
            self.timeout,
            self._waiting_data,
        ) 

    def open(self):
        """Open a (previously initialized) port on dummy_serial."""
        if VERBOSE:
            _print_out('\nDummy_serial: Opening port\n')

        if self._isOpen:
            raise IOError('Dummy_serial: The port is already open')
            
        self._isOpen = True
        self.port = self.initial_port_name
        self.ser = pyb.UART(self.port,self.baudrate)

    def close(self):
        """Close a port on dummy_serial."""
        if VERBOSE:
            _print_out('\nDummy_serial: Closing port\n')

        if not self._isOpen:
            raise IOError('Dummy_serial: The port is already closed')
            
        self._isOpen = False
        self.port = None
        self.ser.close()
        self.ser = None

    def write(self, inputdata):
        if VERBOSE:
            _print_out('\nDummy_serial: Writing to port. Given:' + repr(inputdata) + '\n')
            
        if sys.version_info[0] > 2:
            if not type(inputdata) == bytes:
                raise TypeError('The input must be type bytes. Given:' + repr(inputdata))
            inputstring = str(inputdata, encoding='latin1')
        else:
            inputstring = inputdata

        if not self._isOpen:
            raise IOError('Dummy_serial: Trying to write, but the port is not open. Given:' + repr(inputdata))

        # Look up which data that should be waiting for subsequent read commands
        print('write data to serial port:' + str(inputdata))
        self.ser.write(inputdata)
        
    def read(self, numberOfBytes):
        if VERBOSE:
            _print_out('\nDummy_serial: Reading from port (max length {!r} bytes)'.format(numberOfBytes))
        
        if numberOfBytes < 0:
            raise IOError('Dummy_serial: The numberOfBytes to read must not be negative. Given: {!r}'.format(numberOfBytes))
        
        if not self._isOpen:
            raise IOError('Dummy_serial: Trying to read, but the port is not open.')

        returnstring = self.ser.read(numberOfBytes)
        if len(returnstring) == 0:
            return bytes()
        else:
            return bytes(returnstring, encoding='latin1')            


def _print_out( inputstring ):
    """Print the inputstring. To make it compatible with Python2 and Python3."""
    sys.stdout.write(inputstring + '\n')

