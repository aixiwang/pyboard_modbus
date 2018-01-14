#!/usr/bin/env python
#----------------------------------------------------------
# ported from https://github.com/pyhys/minimalmodbus
#
# Ported by Aixi Wang (aixi.wang@hotmail.com)
#----------------------------------------------------------
import os
import pyboard_serial
import struct
import sys
import time
if sys.version > '3':
    import binascii
if sys.version > '3':
    long = int


    
_NUMBER_OF_BYTES_PER_REGISTER = 2
_SECONDS_TO_MILLISECONDS = 1000
_ASCII_HEADER = ':'
_ASCII_FOOTER = '\r\n'

# Several instrument instances can share the same serialport
_SERIALPORTS = {}
_LATEST_READ_TIMES = {}

####################
## Default values ##
####################
BAUDRATE = 9600
PARITY   = pyboard_serial.PARITY_NONE
BYTESIZE = 8
STOPBITS = 1
TIMEOUT  = 0.05
CLOSE_PORT_AFTER_EACH_CALL = False


#####################
## Named constants ##
#####################

MODE_RTU   = 'rtu'
MODE_ASCII = 'ascii'

##############################
## PyboardModbus Class             ##
##############################


class PybModbus():
    def __init__(self, port, slaveaddress, mode=MODE_RTU):
        if port not in _SERIALPORTS or not _SERIALPORTS[port]:
            self.serial = _SERIALPORTS[port] = pyboard_serial.Serial(port=port, baudrate=BAUDRATE, parity=PARITY, bytesize=BYTESIZE, stopbits=STOPBITS, timeout=TIMEOUT)
        else:
            self.serial = _SERIALPORTS[port]
            if self.serial.port is None:
                self.serial.open()

        self.address = slaveaddress

        self.mode = mode


        self.debug = False

        self.close_port_after_each_call = CLOSE_PORT_AFTER_EACH_CALL

        self.precalculate_read_size = True
        
        self.handle_local_echo = False


        if  self.close_port_after_each_call:
            self.serial.close()

    def __repr__(self):
        return "{}.{}<id=0x{:x}, address={}, mode={}, close_port_after_each_call={}, precalculate_read_size={}, debug={}, serial={}>".format(
            self.__module__,
            self.__class__.__name__,
            id(self),
            self.address,
            self.mode,
            self.close_port_after_each_call,
            self.precalculate_read_size,
            self.debug,
            self.serial,
            )

    ######################################
    ## Methods for talking to the slave ##
    ######################################


    def read_bit(self, registeraddress, functioncode=2):
        _checkFunctioncode(functioncode, [1, 2])
        return self._genericCommand(functioncode, registeraddress)


    def write_bit(self, registeraddress, value, functioncode=5):
        _checkFunctioncode(functioncode, [5, 15])
        _checkInt(value, minvalue=0, maxvalue=1, description='input value')
        self._genericCommand(functioncode, registeraddress, value)


    def read_register(self, registeraddress, numberOfDecimals=0, functioncode=3, signed=False):
        _checkFunctioncode(functioncode, [3, 4])
        _checkInt(numberOfDecimals, minvalue=0, maxvalue=10, description='number of decimals')
        _checkBool(signed, description='signed')
        return self._genericCommand(functioncode, registeraddress, numberOfDecimals=numberOfDecimals, signed=signed)


    def write_register(self, registeraddress, value, numberOfDecimals=0, functioncode=16, signed=False):
        _checkFunctioncode(functioncode, [6, 16])
        _checkInt(numberOfDecimals, minvalue=0, maxvalue=10, description='number of decimals')
        _checkBool(signed, description='signed')
        _checkNumerical(value, description='input value')

        self._genericCommand(functioncode, registeraddress, value, numberOfDecimals, signed=signed)


    def read_long(self, registeraddress, functioncode=3, signed=False):
        _checkFunctioncode(functioncode, [3, 4])
        _checkBool(signed, description='signed')
        return self._genericCommand(functioncode, registeraddress, numberOfRegisters=2, signed=signed, payloadformat='long')


    def write_long(self, registeraddress, value, signed=False):
        MAX_VALUE_LONG =  4294967295  # Unsigned INT32
        MIN_VALUE_LONG = -2147483648  # INT32

        _checkInt(value, minvalue=MIN_VALUE_LONG, maxvalue=MAX_VALUE_LONG, description='input value')
        _checkBool(signed, description='signed')
        self._genericCommand(16, registeraddress, value, numberOfRegisters=2, signed=signed, payloadformat='long')


    def read_float(self, registeraddress, functioncode=3, numberOfRegisters=2):
        _checkFunctioncode(functioncode, [3, 4])
        _checkInt(numberOfRegisters, minvalue=2, maxvalue=4, description='number of registers')
        return self._genericCommand(functioncode, registeraddress, numberOfRegisters=numberOfRegisters, payloadformat='float')


    def write_float(self, registeraddress, value, numberOfRegisters=2):
        _checkNumerical(value, description='input value')
        _checkInt(numberOfRegisters, minvalue=2, maxvalue=4, description='number of registers')
        self._genericCommand(16, registeraddress, value, \
            numberOfRegisters=numberOfRegisters, payloadformat='float')


    def read_string(self, registeraddress, numberOfRegisters=16, functioncode=3):
        _checkFunctioncode(functioncode, [3, 4])
        _checkInt(numberOfRegisters, minvalue=1, description='number of registers for read string')
        return self._genericCommand(functioncode, registeraddress, \
            numberOfRegisters=numberOfRegisters, payloadformat='string')


    def write_string(self, registeraddress, textstring, numberOfRegisters=16):
        _checkInt(numberOfRegisters, minvalue=1, description='number of registers for write string')
        _checkString(textstring, 'input string', minlength=1, maxlength=2 * numberOfRegisters)
        self._genericCommand(16, registeraddress, textstring, \
            numberOfRegisters=numberOfRegisters, payloadformat='string')


    def read_registers(self, registeraddress, numberOfRegisters, functioncode=3):
        _checkFunctioncode(functioncode, [3, 4])
        _checkInt(numberOfRegisters, minvalue=1, description='number of registers')
        return self._genericCommand(functioncode, registeraddress, \
            numberOfRegisters=numberOfRegisters, payloadformat='registers')


    def write_registers(self, registeraddress, values):
        if not isinstance(values, list):
            raise TypeError('The "values parameter" must be a list. Given: {0!r}'.format(values))
        _checkInt(len(values), minvalue=1, description='length of input list')
        # Note: The content of the list is checked at content conversion.

        self._genericCommand(16, registeraddress, values, numberOfRegisters=len(values), payloadformat='registers')

    #####################
    ## Generic command ##
    #####################


    def _genericCommand(self, functioncode, registeraddress, value=None, \
            numberOfDecimals=0, numberOfRegisters=1, signed=False, payloadformat=None):
        NUMBER_OF_BITS = 1
        NUMBER_OF_BYTES_FOR_ONE_BIT = 1
        NUMBER_OF_BYTES_BEFORE_REGISTERDATA = 1
        ALL_ALLOWED_FUNCTIONCODES = list(range(1, 7)) + [15, 16]  # To comply with both Python2 and Python3
        MAX_NUMBER_OF_REGISTERS = 255

        # Payload format constants, so datatypes can be told apart.
        # Note that bit datatype not is included, because it uses other functioncodes.
        PAYLOADFORMAT_LONG      = 'long'
        PAYLOADFORMAT_FLOAT     = 'float'
        PAYLOADFORMAT_STRING    = 'string'
        PAYLOADFORMAT_REGISTER  = 'register'
        PAYLOADFORMAT_REGISTERS = 'registers'

        ALL_PAYLOADFORMATS = [PAYLOADFORMAT_LONG, PAYLOADFORMAT_FLOAT, \
            PAYLOADFORMAT_STRING, PAYLOADFORMAT_REGISTER, PAYLOADFORMAT_REGISTERS]

        ## Check input values ##
        _checkFunctioncode(functioncode, ALL_ALLOWED_FUNCTIONCODES)  # Note: The calling facade functions should validate this
        _checkRegisteraddress(registeraddress)
        _checkInt(numberOfDecimals, minvalue=0, description='number of decimals')
        _checkInt(numberOfRegisters, minvalue=1, maxvalue=MAX_NUMBER_OF_REGISTERS, description='number of registers')
        _checkBool(signed, description='signed')

        if payloadformat is not None:
            if payloadformat not in ALL_PAYLOADFORMATS:
                raise ValueError('Wrong payload format variable. Given: {0!r}'.format(payloadformat))

        ## Check combinations of input parameters ##
        numberOfRegisterBytes = numberOfRegisters * _NUMBER_OF_BYTES_PER_REGISTER

                    # Payload format
        if functioncode in [3, 4, 6, 16] and payloadformat is None:
            payloadformat = PAYLOADFORMAT_REGISTER

        if functioncode in [3, 4, 6, 16]:
            if payloadformat not in ALL_PAYLOADFORMATS:
                raise ValueError('The payload format is unknown. Given format: {0!r}, functioncode: {1!r}.'.\
                    format(payloadformat, functioncode))
        else:
            if payloadformat is not None:
                raise ValueError('The payload format given is not allowed for this function code. ' + \
                    'Given format: {0!r}, functioncode: {1!r}.'.format(payloadformat, functioncode))

                    # Signed and numberOfDecimals
        if signed:
            if payloadformat not in [PAYLOADFORMAT_REGISTER, PAYLOADFORMAT_LONG]:
                raise ValueError('The "signed" parameter can not be used for this data format. ' + \
                    'Given format: {0!r}.'.format(payloadformat))

        if numberOfDecimals > 0 and payloadformat != PAYLOADFORMAT_REGISTER:
            raise ValueError('The "numberOfDecimals" parameter can not be used for this data format. ' + \
                'Given format: {0!r}.'.format(payloadformat))

                    # Number of registers
        if functioncode not in [3, 4, 16] and numberOfRegisters != 1:
            raise ValueError('The numberOfRegisters is not valid for this function code. ' + \
                'NumberOfRegisters: {0!r}, functioncode {1}.'.format(numberOfRegisters, functioncode))

        if functioncode == 16 and payloadformat == PAYLOADFORMAT_REGISTER and numberOfRegisters != 1:
            raise ValueError('Wrong numberOfRegisters when writing to a ' + \
                'single register. Given {0!r}.'.format(numberOfRegisters))
            # Note: For function code 16 there is checking also in the content conversion functions.

                    # Value
        if functioncode in [5, 6, 15, 16] and value is None:
            raise ValueError('The input value is not valid for this function code. ' + \
                'Given {0!r} and {1}.'.format(value, functioncode))

        if functioncode == 16 and payloadformat in [PAYLOADFORMAT_REGISTER, PAYLOADFORMAT_FLOAT, PAYLOADFORMAT_LONG]:
            _checkNumerical(value, description='input value')

        if functioncode == 6 and payloadformat == PAYLOADFORMAT_REGISTER:
            _checkNumerical(value, description='input value')

                    # Value for string
        if functioncode == 16 and payloadformat == PAYLOADFORMAT_STRING:
            _checkString(value, 'input string', minlength=1, maxlength=numberOfRegisterBytes)
            # Note: The string might be padded later, so the length might be shorter than numberOfRegisterBytes.

                    # Value for registers
        if functioncode == 16 and payloadformat == PAYLOADFORMAT_REGISTERS:
            if not isinstance(value, list):
                raise TypeError('The value parameter must be a list. Given {0!r}.'.format(value))

            if len(value) != numberOfRegisters:
                raise ValueError('The list length does not match number of registers. ' + \
                    'List: {0!r},  Number of registers: {1!r}.'.format(value, numberOfRegisters))

        ## Build payload to slave ##
        if functioncode in [1, 2]:
            payloadToSlave = _numToTwoByteString(registeraddress) + \
                            _numToTwoByteString(NUMBER_OF_BITS)

        elif functioncode in [3, 4]:
            payloadToSlave = _numToTwoByteString(registeraddress) + \
                            _numToTwoByteString(numberOfRegisters)

        elif functioncode == 5:
            payloadToSlave = _numToTwoByteString(registeraddress) + \
                            _createBitpattern(functioncode, value)

        elif functioncode == 6:
            payloadToSlave = _numToTwoByteString(registeraddress) + \
                            _numToTwoByteString(value, numberOfDecimals, signed=signed)

        elif functioncode == 15:
            payloadToSlave = _numToTwoByteString(registeraddress) + \
                            _numToTwoByteString(NUMBER_OF_BITS) + \
                            _numToOneByteString(NUMBER_OF_BYTES_FOR_ONE_BIT) + \
                            _createBitpattern(functioncode, value)

        elif functioncode == 16:
            if payloadformat == PAYLOADFORMAT_REGISTER:
                registerdata = _numToTwoByteString(value, numberOfDecimals, signed=signed)

            elif payloadformat == PAYLOADFORMAT_STRING:
                registerdata = _textstringToBytestring(value, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_LONG:
                registerdata = _longToBytestring(value, signed, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_FLOAT:
                registerdata = _floatToBytestring(value, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_REGISTERS:
                registerdata = _valuelistToBytestring(value, numberOfRegisters)

            assert len(registerdata) == numberOfRegisterBytes
            payloadToSlave = _numToTwoByteString(registeraddress) + \
                            _numToTwoByteString(numberOfRegisters) + \
                            _numToOneByteString(numberOfRegisterBytes) + \
                            registerdata

        ## Communicate ##
        payloadFromSlave = self._performCommand(functioncode, payloadToSlave)

        ## Check the contents in the response payload ##
        if functioncode in [1, 2, 3, 4]:
            _checkResponseByteCount(payloadFromSlave)  # response byte count

        if functioncode in [5, 6, 15, 16]:
            _checkResponseRegisterAddress(payloadFromSlave, registeraddress)  # response register address

        if functioncode == 5:
            _checkResponseWriteData(payloadFromSlave, _createBitpattern(functioncode, value))  # response write data

        if functioncode == 6:
            _checkResponseWriteData(payloadFromSlave, \
                _numToTwoByteString(value, numberOfDecimals, signed=signed))  # response write data

        if functioncode == 15:
            _checkResponseNumberOfRegisters(payloadFromSlave, NUMBER_OF_BITS)  # response number of bits

        if functioncode == 16:
            _checkResponseNumberOfRegisters(payloadFromSlave, numberOfRegisters)  # response number of registers

        ## Calculate return value ##
        if functioncode in [1, 2]:
            registerdata = payloadFromSlave[NUMBER_OF_BYTES_BEFORE_REGISTERDATA:]
            if len(registerdata) != NUMBER_OF_BYTES_FOR_ONE_BIT:
                raise ValueError('The registerdata length does not match NUMBER_OF_BYTES_FOR_ONE_BIT. ' + \
                    'Given {0}.'.format(len(registerdata)))

            return _bitResponseToValue(registerdata)

        if functioncode in [3, 4]:
            registerdata = payloadFromSlave[NUMBER_OF_BYTES_BEFORE_REGISTERDATA:]
            if len(registerdata) != numberOfRegisterBytes:
                raise ValueError('The registerdata length does not match number of register bytes. ' + \
                    'Given {0!r} and {1!r}.'.format(len(registerdata), numberOfRegisterBytes))

            if payloadformat == PAYLOADFORMAT_STRING:
                return _bytestringToTextstring(registerdata, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_LONG:
                return _bytestringToLong(registerdata, signed, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_FLOAT:
                return _bytestringToFloat(registerdata, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_REGISTERS:
                return _bytestringToValuelist(registerdata, numberOfRegisters)

            elif payloadformat == PAYLOADFORMAT_REGISTER:
                return _twoByteStringToNum(registerdata, numberOfDecimals, signed=signed)

            raise ValueError('Wrong payloadformat for return value generation. ' + \
                'Given {0}'.format(payloadformat))

    ##########################################
    ## Communication implementation details ##
    ##########################################


    def _performCommand(self, functioncode, payloadToSlave):
        DEFAULT_NUMBER_OF_BYTES_TO_READ = 1000

        _checkFunctioncode(functioncode, None)
        _checkString(payloadToSlave, description='payload')

        # Build request
        request = _embedPayload(self.address, self.mode, functioncode, payloadToSlave)

        # Calculate number of bytes to read
        number_of_bytes_to_read = DEFAULT_NUMBER_OF_BYTES_TO_READ
        if self.precalculate_read_size:
            try:
                number_of_bytes_to_read = _predictResponseSize(self.mode, functioncode, payloadToSlave)
            except:
                if self.debug:
                    template = 'MinimalModbus debug mode. Could not precalculate response size for Modbus {} mode. ' + \
                        'Will read {} bytes. request: {!r}'
                    _print_out(template.format(self.mode, number_of_bytes_to_read, request))

        # Communicate
        response = self._communicate(request, number_of_bytes_to_read)

        # Extract payload
        payloadFromSlave = _extractPayload(response, self.address, self.mode, functioncode)
        return payloadFromSlave


    def _communicate(self, request, number_of_bytes_to_read):
        _checkString(request, minlength=1, description='request')
        _checkInt(number_of_bytes_to_read)

        if self.debug:
            _print_out('\nMinimalModbus debug mode. Writing to instrument (expecting {} bytes back): {!r} ({})'. \
                format(number_of_bytes_to_read, request, _hexlify(request)))

        if self.close_port_after_each_call:
            self.serial.open()

        #self.serial.flushInput() TODO

        if sys.version_info[0] > 2:
            request = bytes(request, encoding='latin1')  # Convert types to make it Python3 compatible

        # Sleep to make sure 3.5 character times have passed
        minimum_silent_period   = _calculate_minimum_silent_period(self.serial.baudrate)
        time_since_read         = time.time() - _LATEST_READ_TIMES.get(self.serial.port, 0)

        if time_since_read < minimum_silent_period:
            sleep_time = minimum_silent_period - time_since_read

            if self.debug:
                template = 'MinimalModbus debug mode. Sleeping for {:.1f} ms. ' + \
                        'Minimum silent period: {:.1f} ms, time since read: {:.1f} ms.'
                text = template.format(
                    sleep_time * _SECONDS_TO_MILLISECONDS,
                    minimum_silent_period * _SECONDS_TO_MILLISECONDS,
                    time_since_read * _SECONDS_TO_MILLISECONDS)
                _print_out(text)

            time.sleep(sleep_time)

        elif self.debug:
            template = 'MinimalModbus debug mode. No sleep required before write. ' + \
                'Time since previous read: {:.1f} ms, minimum silent period: {:.2f} ms.'
            text = template.format(
                time_since_read * _SECONDS_TO_MILLISECONDS,
                minimum_silent_period * _SECONDS_TO_MILLISECONDS)
            _print_out(text)

        # Write request
        latest_write_time = time.time()
        
        self.serial.write(request)

        # Read and discard local echo
        if self.handle_local_echo:
            localEchoToDiscard = self.serial.read(len(request))
            if self.debug:
                template = 'MinimalModbus debug mode. Discarding this local echo: {!r} ({} bytes).' 
                text = template.format(localEchoToDiscard, len(localEchoToDiscard))
                _print_out(text)
            if localEchoToDiscard != request:
                template = 'Local echo handling is enabled, but the local echo does not match the sent request. ' + \
                    'Request: {!r} ({} bytes), local echo: {!r} ({} bytes).' 
                text = template.format(request, len(request), localEchoToDiscard, len(localEchoToDiscard))
                raise IOError(text)

        # Read response
        answer = self.serial.read(number_of_bytes_to_read)
        _LATEST_READ_TIMES[self.serial.port] = time.time()

        if self.close_port_after_each_call:
            self.serial.close()

        if sys.version_info[0] > 2:
            answer = str(answer, encoding='latin1')  # Convert types to make it Python3 compatible

        if self.debug:
            template = 'MinimalModbus debug mode. Response from instrument: {!r} ({}) ({} bytes), ' + \
                'roundtrip time: {:.1f} ms. Timeout setting: {:.1f} ms.\n'
            text = template.format(
                answer,
                _hexlify(answer),
                len(answer),
                (_LATEST_READ_TIMES.get(self.serial.port, 0) - latest_write_time) * _SECONDS_TO_MILLISECONDS,
                self.serial.timeout * _SECONDS_TO_MILLISECONDS)
            _print_out(text)

        if len(answer) == 0:
            raise IOError('No communication with the instrument (no answer)')

        return answer

####################
# Payload handling #
####################


def _embedPayload(slaveaddress, mode, functioncode, payloaddata):
    _checkSlaveaddress(slaveaddress)
    _checkMode(mode)
    _checkFunctioncode(functioncode, None)
    _checkString(payloaddata, description='payload')

    firstPart = _numToOneByteString(slaveaddress) + _numToOneByteString(functioncode) + payloaddata

    request = firstPart + _calculateCrcString(firstPart)

    return request


def _extractPayload(response, slaveaddress, mode, functioncode):
    BYTEPOSITION_FOR_ASCII_HEADER          = 0  # Relative to plain response

    BYTEPOSITION_FOR_SLAVEADDRESS          = 0  # Relative to (stripped) response
    BYTEPOSITION_FOR_FUNCTIONCODE          = 1

    NUMBER_OF_RESPONSE_STARTBYTES          = 2  # Number of bytes before the response payload (in stripped response)
    NUMBER_OF_CRC_BYTES                    = 2
    NUMBER_OF_LRC_BYTES                    = 1
    BITNUMBER_FUNCTIONCODE_ERRORINDICATION = 7

    MINIMAL_RESPONSE_LENGTH_RTU            = NUMBER_OF_RESPONSE_STARTBYTES + NUMBER_OF_CRC_BYTES
    MINIMAL_RESPONSE_LENGTH_ASCII          = 9

    # Argument validity testing
    _checkString(response, description='response')
    _checkSlaveaddress(slaveaddress)
    _checkMode(mode)
    _checkFunctioncode(functioncode, None)

    plainresponse = response

    # Validate response length
    if len(response) < MINIMAL_RESPONSE_LENGTH_RTU:
        raise ValueError('Too short Modbus RTU response (minimum length {} bytes). Response: {!r}'.format( \
            MINIMAL_RESPONSE_LENGTH_RTU,
            response))



    # Validate response checksum

    calculateChecksum = _calculateCrcString
    numberOfChecksumBytes = NUMBER_OF_CRC_BYTES

    receivedChecksum = response[-numberOfChecksumBytes:]
    responseWithoutChecksum = response[0 : len(response) - numberOfChecksumBytes]
    calculatedChecksum = calculateChecksum(responseWithoutChecksum)

    if receivedChecksum != calculatedChecksum:
        template = 'Checksum error in {} mode: {!r} instead of {!r} . The response is: {!r} (plain response: {!r})'
        text = template.format(
                mode,
                receivedChecksum,
                calculatedChecksum,
                response, plainresponse)
        raise ValueError(text)

    # Check slave address
    responseaddress = ord(response[BYTEPOSITION_FOR_SLAVEADDRESS])

    if responseaddress != slaveaddress:
        raise ValueError('Wrong return slave address: {} instead of {}. The response is: {!r}'.format( \
            responseaddress, slaveaddress, response))

    # Check function code
    receivedFunctioncode = ord(response[BYTEPOSITION_FOR_FUNCTIONCODE])

    if receivedFunctioncode == _setBitOn(functioncode, BITNUMBER_FUNCTIONCODE_ERRORINDICATION):
        raise ValueError('The slave is indicating an error. The response is: {!r}'.format(response))

    elif receivedFunctioncode != functioncode:
        raise ValueError('Wrong functioncode: {} instead of {}. The response is: {!r}'.format( \
            receivedFunctioncode, functioncode, response))

    # Read data payload
    firstDatabyteNumber = NUMBER_OF_RESPONSE_STARTBYTES


    lastDatabyteNumber = len(response) - NUMBER_OF_CRC_BYTES

    payload = response[firstDatabyteNumber:lastDatabyteNumber]
    return payload

############################################
## Serial communication utility functions ##
############################################


def _predictResponseSize(mode, functioncode, payloadToSlave):
    MIN_PAYLOAD_LENGTH = 4  # For implemented functioncodes here
    BYTERANGE_FOR_GIVEN_SIZE = slice(2, 4)  # Within the payload

    NUMBER_OF_PAYLOAD_BYTES_IN_WRITE_CONFIRMATION = 4
    NUMBER_OF_PAYLOAD_BYTES_FOR_BYTECOUNTFIELD = 1

    RTU_TO_ASCII_PAYLOAD_FACTOR = 2

    NUMBER_OF_RTU_RESPONSE_STARTBYTES   = 2
    NUMBER_OF_RTU_RESPONSE_ENDBYTES     = 2
    NUMBER_OF_ASCII_RESPONSE_STARTBYTES = 5
    NUMBER_OF_ASCII_RESPONSE_ENDBYTES   = 4

    # Argument validity testing
    _checkMode(mode)
    _checkFunctioncode(functioncode, None)
    _checkString(payloadToSlave, description='payload', minlength=MIN_PAYLOAD_LENGTH)

    # Calculate payload size
    if functioncode in [5, 6, 15, 16]:
        response_payload_size = NUMBER_OF_PAYLOAD_BYTES_IN_WRITE_CONFIRMATION

    elif functioncode in [1, 2, 3, 4]:
        given_size = _twoByteStringToNum(payloadToSlave[BYTERANGE_FOR_GIVEN_SIZE])
        if functioncode == 1 or functioncode == 2:
            # Algorithm from MODBUS APPLICATION PROTOCOL SPECIFICATION V1.1b
            number_of_inputs = given_size
            response_payload_size = NUMBER_OF_PAYLOAD_BYTES_FOR_BYTECOUNTFIELD + \
                                    number_of_inputs // 8 + (1 if number_of_inputs % 8 else 0)

        elif functioncode == 3 or functioncode == 4:
            number_of_registers = given_size
            response_payload_size = NUMBER_OF_PAYLOAD_BYTES_FOR_BYTECOUNTFIELD + \
                                    number_of_registers * _NUMBER_OF_BYTES_PER_REGISTER

    else:
        raise ValueError('Wrong functioncode: {}. The payload is: {!r}'.format( \
            functioncode, payloadToSlave))

    # Calculate number of bytes to read

    return NUMBER_OF_RTU_RESPONSE_STARTBYTES + \
        response_payload_size + \
        NUMBER_OF_RTU_RESPONSE_ENDBYTES


def _calculate_minimum_silent_period(baudrate):
    _checkNumerical(baudrate, minvalue=1, description='baudrate')  # Avoid division by zero

    BITTIMES_PER_CHARACTERTIME = 11
    MINIMUM_SILENT_CHARACTERTIMES = 3.5

    bittime = 1 / float(baudrate)
    return bittime * BITTIMES_PER_CHARACTERTIME * MINIMUM_SILENT_CHARACTERTIMES

##############################
# String and num conversions #
##############################


def _numToOneByteString(inputvalue):
    _checkInt(inputvalue, minvalue=0, maxvalue=0xFF)

    return chr(inputvalue)


def _numToTwoByteString(value, numberOfDecimals=0, LsbFirst=False, signed=False):
    _checkNumerical(value, description='inputvalue')
    _checkInt(numberOfDecimals, minvalue=0, description='number of decimals')
    _checkBool(LsbFirst, description='LsbFirst')
    _checkBool(signed, description='signed parameter')

    multiplier = 10 ** numberOfDecimals
    integer = int(float(value) * multiplier)

    if LsbFirst:
        formatcode = '<'  # Little-endian
    else:
        formatcode = '>'  # Big-endian
    if signed:
        formatcode += 'h'  # (Signed) short (2 bytes)
    else:
        formatcode += 'H'  # Unsigned short (2 bytes)

    outstring = _pack(formatcode, integer)
    assert len(outstring) == 2
    return outstring


def _twoByteStringToNum(bytestring, numberOfDecimals=0, signed=False):
    _checkString(bytestring, minlength=2, maxlength=2, description='bytestring')
    _checkInt(numberOfDecimals, minvalue=0, description='number of decimals')
    _checkBool(signed, description='signed parameter')

    formatcode = '>'  # Big-endian
    if signed:
        formatcode += 'h'  # (Signed) short (2 bytes)
    else:
        formatcode += 'H'  # Unsigned short (2 bytes)

    fullregister = _unpack(formatcode, bytestring)

    if numberOfDecimals == 0:
        return fullregister
    divisor = 10 ** numberOfDecimals
    return fullregister / float(divisor)


def _longToBytestring(value, signed=False, numberOfRegisters=2):
    _checkInt(value, description='inputvalue')
    _checkBool(signed, description='signed parameter')
    _checkInt(numberOfRegisters, minvalue=2, maxvalue=2, description='number of registers')

    formatcode = '>'  # Big-endian
    if signed:
        formatcode += 'l'  # (Signed) long (4 bytes)
    else:
        formatcode += 'L'  # Unsigned long (4 bytes)

    outstring = _pack(formatcode, value)
    assert len(outstring) == 4
    return outstring


def _bytestringToLong(bytestring, signed=False, numberOfRegisters=2):
    _checkString(bytestring, 'byte string', minlength=4, maxlength=4)
    _checkBool(signed, description='signed parameter')
    _checkInt(numberOfRegisters, minvalue=2, maxvalue=2, description='number of registers')

    formatcode = '>'  # Big-endian
    if signed:
        formatcode += 'l'  # (Signed) long (4 bytes)
    else:
        formatcode += 'L'  # Unsigned long (4 bytes)

    return _unpack(formatcode, bytestring)


def _floatToBytestring(value, numberOfRegisters=2):
    _checkNumerical(value, description='inputvalue')
    _checkInt(numberOfRegisters, minvalue=2, maxvalue=4, description='number of registers')

    formatcode = '>'  # Big-endian
    if numberOfRegisters == 2:
        formatcode += 'f'  # Float (4 bytes)
        lengthtarget = 4
    elif numberOfRegisters == 4:
        formatcode += 'd'  # Double (8 bytes)
        lengthtarget = 8
    else:
        raise ValueError('Wrong number of registers! Given value is {0!r}'.format(numberOfRegisters))

    outstring = _pack(formatcode, value)
    assert len(outstring) == lengthtarget
    return outstring


def _bytestringToFloat(bytestring, numberOfRegisters=2):
    _checkString(bytestring, minlength=4, maxlength=8, description='bytestring')
    _checkInt(numberOfRegisters, minvalue=2, maxvalue=4, description='number of registers')

    numberOfBytes = _NUMBER_OF_BYTES_PER_REGISTER * numberOfRegisters

    formatcode = '>'  # Big-endian
    if numberOfRegisters == 2:
        formatcode += 'f'  # Float (4 bytes)
    elif numberOfRegisters == 4:
        formatcode += 'd'  # Double (8 bytes)
    else:
        raise ValueError('Wrong number of registers! Given value is {0!r}'.format(numberOfRegisters))

    if len(bytestring) != numberOfBytes:
        raise ValueError('Wrong length of the byte string! Given value is {0!r}, and numberOfRegisters is {1!r}.'.\
            format(bytestring, numberOfRegisters))

    return _unpack(formatcode, bytestring)


def _textstringToBytestring(inputstring, numberOfRegisters=16):
    _checkInt(numberOfRegisters, minvalue=1, description='number of registers')
    maxCharacters = _NUMBER_OF_BYTES_PER_REGISTER * numberOfRegisters
    _checkString(inputstring, 'input string', minlength=1, maxlength=maxCharacters)

    bytestring = inputstring.ljust(maxCharacters)  # Pad with space
    assert len(bytestring) == maxCharacters
    return bytestring


def _bytestringToTextstring(bytestring, numberOfRegisters=16):
    _checkInt(numberOfRegisters, minvalue=1, description='number of registers')
    maxCharacters = _NUMBER_OF_BYTES_PER_REGISTER * numberOfRegisters
    _checkString(bytestring, 'byte string', minlength=maxCharacters, maxlength=maxCharacters)

    textstring = bytestring
    return textstring


def _valuelistToBytestring(valuelist, numberOfRegisters):
    MINVALUE = 0
    MAXVALUE = 65535

    _checkInt(numberOfRegisters, minvalue=1, description='number of registers')

    if not isinstance(valuelist, list):
        raise TypeError('The valuelist parameter must be a list. Given {0!r}.'.format(valuelist))

    for value in valuelist:
        _checkInt(value, minvalue=MINVALUE, maxvalue=MAXVALUE, description='elements in the input value list')

    _checkInt(len(valuelist), minvalue=numberOfRegisters, maxvalue=numberOfRegisters, \
        description='length of the list')

    numberOfBytes = _NUMBER_OF_BYTES_PER_REGISTER * numberOfRegisters

    bytestring = ''
    for value in valuelist:
        bytestring += _numToTwoByteString(value, signed=False)

    assert len(bytestring) == numberOfBytes
    return bytestring


def _bytestringToValuelist(bytestring, numberOfRegisters):
    _checkInt(numberOfRegisters, minvalue=1, description='number of registers')
    numberOfBytes = _NUMBER_OF_BYTES_PER_REGISTER * numberOfRegisters
    _checkString(bytestring, 'byte string', minlength=numberOfBytes, maxlength=numberOfBytes)

    values = []
    for i in range(numberOfRegisters):
        offset = _NUMBER_OF_BYTES_PER_REGISTER * i
        substring = bytestring[offset : offset + _NUMBER_OF_BYTES_PER_REGISTER]
        values.append(_twoByteStringToNum(substring))

    return values


def _pack(formatstring, value):
    _checkString(formatstring, description='formatstring', minlength=1)

    try:
        result = struct.pack(formatstring, value)
    except:
        errortext = 'The value to send is probably out of range, as the num-to-bytestring conversion failed.'
        errortext += ' Value: {0!r} Struct format code is: {1}'
        raise ValueError(errortext.format(value, formatstring))

    if sys.version_info[0] > 2:
        return str(result, encoding='latin1')  # Convert types to make it Python3 compatible
    return result


def _unpack(formatstring, packed):
    _checkString(formatstring, description='formatstring', minlength=1)
    _checkString(packed, description='packed string', minlength=1)

    if sys.version_info[0] > 2:
        packed = bytes(packed, encoding='latin1')  # Convert types to make it Python3 compatible

    try:
        value = struct.unpack(formatstring, packed)[0]
    except:
        errortext = 'The received bytestring is probably wrong, as the bytestring-to-num conversion failed.'
        errortext += ' Bytestring: {0!r} Struct format code is: {1}'
        raise ValueError(errortext.format(packed, formatstring))

    return value


def _hexencode(bytestring, insert_spaces = False):
    _checkString(bytestring, description='byte string')

    separator = '' if not insert_spaces else ' '
    
    # Use plain string formatting instead of binhex.hexlify,
    # in order to have it Python 2.x and 3.x compatible

    byte_representions = []
    for c in bytestring:
        byte_representions.append( '{0:02X}'.format(ord(c)) )
    return separator.join(byte_representions).strip()


def _hexdecode(hexstring):
    # Note: For Python3 the appropriate would be: raise TypeError(new_error_message) from err
    # but the Python2 interpreter will indicate SyntaxError.
    # Thus we need to live with this warning in Python3:
    # 'During handling of the above exception, another exception occurred'

    _checkString(hexstring, description='hexstring')

    if len(hexstring) % 2 != 0:
        raise ValueError('The input hexstring must be of even length. Given: {!r}'.format(hexstring))

    if sys.version_info[0] > 2:
        by = bytes(hexstring, 'latin1')
        try:
            return str(binascii.unhexlify(by), encoding='latin1')
        except binascii.Error as err:
            new_error_message = 'Hexdecode reported an error: {!s}. Input hexstring: {}'.format(err.args[0], hexstring)
            raise TypeError(new_error_message)

    else:
        try:
            return hexstring.decode('hex')
        except TypeError as err:
            raise TypeError('Hexdecode reported an error: {}. Input hexstring: {}'.format(err.message, hexstring))


def _hexlify(bytestring):
    return _hexencode(bytestring, insert_spaces = True)


def _bitResponseToValue(bytestring):
    _checkString(bytestring, description='bytestring', minlength=1, maxlength=1)

    RESPONSE_ON  = '\x01'
    RESPONSE_OFF = '\x00'

    if bytestring == RESPONSE_ON:
        return 1
    elif bytestring == RESPONSE_OFF:
        return 0
    else:
        raise ValueError('Could not convert bit response to a value. Input: {0!r}'.format(bytestring))


def _createBitpattern(functioncode, value):
    _checkFunctioncode(functioncode, [5, 15])
    _checkInt(value, minvalue=0, maxvalue=1, description='inputvalue')

    if functioncode == 5:
        if value == 0:
            return '\x00\x00'
        else:
            return '\xff\x00'

    elif functioncode == 15:
        if value == 0:
            return '\x00'
        else:
            return '\x01'  # Is this correct??

#######################
# Number manipulation #
#######################


def _twosComplement(x, bits=16):
    _checkInt(bits, minvalue=0, description='number of bits')
    _checkInt(x, description='input')
    upperlimit = 2 ** (bits - 1) - 1
    lowerlimit = -2 ** (bits - 1)
    if x > upperlimit or x < lowerlimit:
        raise ValueError('The input value is out of range. Given value is {0}, but allowed range is {1} to {2} when using {3} bits.' \
            .format(x, lowerlimit, upperlimit, bits))

    # Calculate two'2 complement
    if x >= 0:
        return x
    return x + 2 ** bits


def _fromTwosComplement(x, bits=16):
    _checkInt(bits, minvalue=0, description='number of bits')

    _checkInt(x, description='input')
    upperlimit = 2 ** (bits) - 1
    lowerlimit = 0
    if x > upperlimit or x < lowerlimit:
        raise ValueError('The input value is out of range. Given value is {0}, but allowed range is {1} to {2} when using {3} bits.' \
            .format(x, lowerlimit, upperlimit, bits))

    # Calculate inverse(?) of two'2 complement
    limit = 2 ** (bits - 1) - 1
    if x <= limit:
        return x
    return x - 2 ** bits

####################
# Bit manipulation #
####################

def _setBitOn(x, bitNum):
    _checkInt(x, minvalue=0, description='input value')
    _checkInt(bitNum, minvalue=0, description='bitnumber')

    return x | (1 << bitNum)

############################
# Error checking functions #
############################

_CRC16TABLE = (
        0, 49345, 49537,   320, 49921,   960,   640, 49729, 50689,  1728,  1920, 
    51009,  1280, 50625, 50305,  1088, 52225,  3264,  3456, 52545,  3840, 53185, 
    52865,  3648,  2560, 51905, 52097,  2880, 51457,  2496,  2176, 51265, 55297, 
     6336,  6528, 55617,  6912, 56257, 55937,  6720,  7680, 57025, 57217,  8000, 
    56577,  7616,  7296, 56385,  5120, 54465, 54657,  5440, 55041,  6080,  5760, 
    54849, 53761,  4800,  4992, 54081,  4352, 53697, 53377,  4160, 61441, 12480, 
    12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721, 
    13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 
    64001, 15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240, 59585, 59777, 
    10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160, 61249, 11520, 60865, 
    60545, 11328, 58369,  9408,  9600, 58689,  9984, 59329, 59009,  9792,  8704, 
    58049, 58241,  9024, 57601,  8640,  8320, 57409, 40961, 24768, 24960, 41281, 
    25344, 41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241, 26048, 25728, 
    42049, 27648, 44225, 44417, 27968, 44801, 28608, 28288, 44609, 43521, 27328, 
    27520, 43841, 26880, 43457, 43137, 26688, 30720, 47297, 47489, 31040, 47873, 
    31680, 31360, 47681, 48641, 32448, 32640, 48961, 32000, 48577, 48257, 31808, 
    46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 
    29504, 45313, 29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 
    21120, 37441, 38401, 22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937, 
    23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040, 39617, 39809, 23360, 
    39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457, 
    19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 
    34177, 17728, 34561, 18368, 18048, 34369, 33281, 17088, 17280, 33601, 16640, 
    33217, 32897, 16448)


def _calculateCrcString(inputstring):
    _checkString(inputstring, description='input CRC string')
 
    # Preload a 16-bit register with ones
    register = 0xFFFF

    for char in inputstring:
        register = (register >> 8) ^ _CRC16TABLE[(register ^ ord(char)) & 0xFF]
 
    return _numToTwoByteString(register, LsbFirst=True)


def _calculateLrcString(inputstring):
    _checkString(inputstring, description='input LRC string')

    register = 0
    for character in inputstring:
        register += ord(character)

    lrc = ((register ^ 0xFF) + 1) & 0xFF

    lrcString = _numToOneByteString(lrc)
    return lrcString


def _checkMode(mode):
    if not isinstance(mode, str):
        raise TypeError('The {0} should be a string. Given: {1!r}'.format("mode", mode))

    if mode not in [MODE_RTU]:
        raise ValueError("Unreconized Modbus mode given. Must be 'rtu' or 'ascii' but {0!r} was given.".format(mode))


def _checkFunctioncode(functioncode, listOfAllowedValues=[]):
    FUNCTIONCODE_MIN = 1
    FUNCTIONCODE_MAX = 127

    _checkInt(functioncode, FUNCTIONCODE_MIN, FUNCTIONCODE_MAX, description='functioncode')

    if listOfAllowedValues is None:
        return

    if not isinstance(listOfAllowedValues, list):
        raise TypeError('The listOfAllowedValues should be a list. Given: {0!r}'.format(listOfAllowedValues))

    for value in listOfAllowedValues:
        _checkInt(value, FUNCTIONCODE_MIN, FUNCTIONCODE_MAX, description='functioncode inside listOfAllowedValues')

    if functioncode not in listOfAllowedValues:
        raise ValueError('Wrong function code: {0}, allowed values are {1!r}'.format(functioncode, listOfAllowedValues))


def _checkSlaveaddress(slaveaddress):
    SLAVEADDRESS_MAX = 247
    SLAVEADDRESS_MIN = 0

    _checkInt(slaveaddress, SLAVEADDRESS_MIN, SLAVEADDRESS_MAX, description='slaveaddress')


def _checkRegisteraddress(registeraddress):
    REGISTERADDRESS_MAX = 0xFFFF
    REGISTERADDRESS_MIN = 0

    _checkInt(registeraddress, REGISTERADDRESS_MIN, REGISTERADDRESS_MAX, description='registeraddress')


def _checkResponseByteCount(payload):
    POSITION_FOR_GIVEN_NUMBER = 0
    NUMBER_OF_BYTES_TO_SKIP = 1

    _checkString(payload, minlength=1, description='payload')

    givenNumberOfDatabytes = ord(payload[POSITION_FOR_GIVEN_NUMBER])
    countedNumberOfDatabytes = len(payload) - NUMBER_OF_BYTES_TO_SKIP

    if givenNumberOfDatabytes != countedNumberOfDatabytes:
        errortemplate = 'Wrong given number of bytes in the response: {0}, but counted is {1} as data payload length is {2}.' + \
            ' The data payload is: {3!r}'
        errortext = errortemplate.format(givenNumberOfDatabytes, countedNumberOfDatabytes, len(payload), payload)
        raise ValueError(errortext)


def _checkResponseRegisterAddress(payload, registeraddress):
    _checkString(payload, minlength=2, description='payload')
    _checkRegisteraddress(registeraddress)

    BYTERANGE_FOR_STARTADDRESS = slice(0, 2)

    bytesForStartAddress = payload[BYTERANGE_FOR_STARTADDRESS]
    receivedStartAddress = _twoByteStringToNum(bytesForStartAddress)

    if receivedStartAddress != registeraddress:
        raise ValueError('Wrong given write start adress: {0}, but commanded is {1}. The data payload is: {2!r}'.format( \
            receivedStartAddress, registeraddress, payload))


def _checkResponseNumberOfRegisters(payload, numberOfRegisters):
    _checkString(payload, minlength=4, description='payload')
    _checkInt(numberOfRegisters, minvalue=1, maxvalue=0xFFFF, description='numberOfRegisters')

    BYTERANGE_FOR_NUMBER_OF_REGISTERS = slice(2, 4)

    bytesForNumberOfRegisters = payload[BYTERANGE_FOR_NUMBER_OF_REGISTERS]
    receivedNumberOfWrittenReisters = _twoByteStringToNum(bytesForNumberOfRegisters)

    if receivedNumberOfWrittenReisters != numberOfRegisters:
        raise ValueError('Wrong number of registers to write in the response: {0}, but commanded is {1}. The data payload is: {2!r}'.format( \
            receivedNumberOfWrittenReisters, numberOfRegisters, payload))


def _checkResponseWriteData(payload, writedata):
    _checkString(payload, minlength=4, description='payload')
    _checkString(writedata, minlength=2, maxlength=2, description='writedata')

    BYTERANGE_FOR_WRITEDATA = slice(2, 4)

    receivedWritedata = payload[BYTERANGE_FOR_WRITEDATA]

    if receivedWritedata != writedata:
        raise ValueError('Wrong write data in the response: {0!r}, but commanded is {1!r}. The data payload is: {2!r}'.format( \
            receivedWritedata, writedata, payload))


def _checkString(inputstring, description, minlength=0, maxlength=None):
    # Type checking
    if not isinstance(description, str):
        raise TypeError('The description should be a string. Given: {0!r}'.format(description))

    if not isinstance(inputstring, str):
        raise TypeError('The {0} should be a string. Given: {1!r}'.format(description, inputstring))

    if not isinstance(maxlength, (int, type(None))):
        raise TypeError('The maxlength must be an integer or None. Given: {0!r}'.format(maxlength))

    # Check values
    _checkInt(minlength, minvalue=0, maxvalue=None, description='minlength')

    if len(inputstring) < minlength:
        raise ValueError('The {0} is too short: {1}, but minimum value is {2}. Given: {3!r}'.format( \
            description, len(inputstring), minlength, inputstring))

    if not maxlength is None:
        if maxlength < 0:
            raise ValueError('The maxlength must be positive. Given: {0}'.format(maxlength))

        if maxlength < minlength:
            raise ValueError('The maxlength must not be smaller than minlength. Given: {0} and {1}'.format( \
                maxlength, minlength))

        if len(inputstring) > maxlength:
            raise ValueError('The {0} is too long: {1}, but maximum value is {2}. Given: {3!r}'.format( \
                description, len(inputstring), maxlength, inputstring))


def _checkInt(inputvalue, minvalue=None, maxvalue=None, description='inputvalue'):
    if not isinstance(description, str):
        raise TypeError('The description should be a string. Given: {0!r}'.format(description))

    if not isinstance(inputvalue, (int, long)):
        raise TypeError('The {0} must be an integer. Given: {1!r}'.format(description, inputvalue))

    if not isinstance(minvalue, (int, long, type(None))):
        raise TypeError('The minvalue must be an integer or None. Given: {0!r}'.format(minvalue))

    if not isinstance(maxvalue, (int, long, type(None))):
        raise TypeError('The maxvalue must be an integer or None. Given: {0!r}'.format(maxvalue))

    _checkNumerical(inputvalue, minvalue, maxvalue, description)


def _checkNumerical(inputvalue, minvalue=None, maxvalue=None, description='inputvalue'):
    # Type checking
    if not isinstance(description, str):
        raise TypeError('The description should be a string. Given: {0!r}'.format(description))

    if not isinstance(inputvalue, (int, long, float)):
        raise TypeError('The {0} must be numerical. Given: {1!r}'.format(description, inputvalue))

    if not isinstance(minvalue, (int, float, long, type(None))):
        raise TypeError('The minvalue must be numeric or None. Given: {0!r}'.format(minvalue))

    if not isinstance(maxvalue, (int, float, long, type(None))):
        raise TypeError('The maxvalue must be numeric or None. Given: {0!r}'.format(maxvalue))

    # Consistency checking
    if (not minvalue is None) and (not maxvalue is None):
        if maxvalue < minvalue:
            raise ValueError('The maxvalue must not be smaller than minvalue. Given: {0} and {1}, respectively.'.format( \
                maxvalue, minvalue))

    # Value checking
    if not minvalue is None:
        if inputvalue < minvalue:
            raise ValueError('The {0} is too small: {1}, but minimum value is {2}.'.format( \
                description, inputvalue, minvalue))

    if not maxvalue is None:
        if inputvalue > maxvalue:
            raise ValueError('The {0} is too large: {1}, but maximum value is {2}.'.format( \
                description, inputvalue, maxvalue))


def _checkBool(inputvalue, description='inputvalue'):
    _checkString(description, minlength=1, description='description string')
    if not isinstance(inputvalue, bool):
        raise TypeError('The {0} must be boolean. Given: {1!r}'.format(description, inputvalue))

#####################
# Development tools #
#####################


def _print_out(inputstring):
    _checkString(inputstring, description='string to print')

    sys.stdout.write(inputstring + '\n')


def _interpretRawMessage(inputstr):
    raise NotImplementedError()
    output = ''
    output += 'Modbus bytestring decoder\n'
    output += 'Input string (length {} characters): {!r} \n'.format(len(inputstr), inputstr)

    # Detect modbus type
    if inputstr.startswith(_ASCII_HEADER) and inputstr.endswith(_ASCII_FOOTER):
        mode = MODE_ASCII
    else:
        mode = MODE_RTU
    output += 'Probably Modbus {} mode.\n'.format(mode.upper())

    # Extract slave address and function code
    try:

        slaveaddress = ord(inputstr[0])
        functioncode = ord(inputstr[1])
        output += 'Slave address: {} (dec). Function code: {} (dec).\n'.format(slaveaddress, functioncode)
    except:
        output += '\nCould not extract slave address and function code. \n\n'

    # Check message validity
    try:
        extractedpayload = _extractPayload(inputstr, slaveaddress, mode, functioncode)
        output += 'Valid message. Extracted payload: {!r}\n'.format(extractedpayload)
    except (ValueError, TypeError) as err:
        output += '\nThe message does not seem to be valid Modbus {}. Error message: \n{}. \n\n'.format(mode.upper(), err.message)
    except NameError as err:
        output += '\nNo message validity checking. \n\n' # Slave address or function code not available

    # Generate table describing the message
    output += '\nPos   Character Hex  Dec  Probable interpretation \n'
    output += '------------------------------------------------- \n'
    for i, character in enumerate(inputstr):
        if i==0:
            description = 'Slave address'
        elif i==1:
            description = 'Function code'
        elif i==len(inputstr)-2:
            description = 'Checksum, CRC LSB'
        elif i==len(inputstr)-1:
            description = 'Checksum, CRC MSB'
        else:
            description = 'Payload'
        output += '{0:3.0f}:  {1!r:<8}  {2:02X}  {2: 4.0f}  {3:<10} \n'.format(i, character, ord(character), description)
    

        
    # Generate description for the payload
    output += '\n\n'
    try:
        output += _interpretPayload(functioncode, extractedpayload)
    except:
        output += '\nCould not interpret the payload. \n\n' # Payload or function code not available
    
    return output
    

def _interpretPayload(functioncode, payload):
    raise NotImplementedError()
    output = ''
    output += 'Modbus payload decoder\n'
    output += 'Input payload (length {} characters): {!r} \n'.format(len(payload), payload)
    output += 'Function code: {} (dec).\n'.format(functioncode)
    
    if len(payload) == 4:
        FourbyteMessageFirstHalfValue = _twoByteStringToNum(payload[0:2])
        FourbyteMessageSecondHalfValue = _twoByteStringToNum(payload[2:4])


    return output

def _getDiagnosticString():
    text = '\n## Diagnostic output from minimalmodbus ## \n\n'
    text += 'Minimalmodbus version: ' + __version__ + '\n'
    text += 'Minimalmodbus status: ' + __status__ + '\n'
    text += 'File name (with relative path): ' + __file__ + '\n'
    text += 'Full file path: ' + os.path.abspath(__file__) + '\n\n'
    text += 'pySerial version: ' + serial.VERSION + '\n'
    text += 'pySerial full file path: ' + os.path.abspath(serial.__file__) + '\n\n'
    text += 'Platform: ' + sys.platform + '\n'
    text += 'Filesystem encoding: ' + repr(sys.getfilesystemencoding()) + '\n'
    text += 'Byteorder: ' + sys.byteorder + '\n'
    text += 'Python version: ' + sys.version + '\n'
    text += 'Python version info: ' + repr(sys.version_info) + '\n'
    text += 'Python flags: ' + repr(sys.flags) + '\n'
    text += 'Python argv: ' + repr(sys.argv) + '\n'
    text += 'Python prefix: ' + repr(sys.prefix) + '\n'
    text += 'Python exec prefix: ' + repr(sys.exec_prefix) + '\n'
    text += 'Python executable: ' + repr(sys.executable) + '\n'
    try:
        text += 'Long info: ' + repr(sys.long_info) + '\n'
    except:
        text += 'Long info: (none)\n'  # For Python3 compatibility
    try:
        text += 'Float repr style: ' + repr(sys.float_repr_style) + '\n\n'
    except:
        text += 'Float repr style: (none) \n\n'  # For Python 2.6 compatibility
    text += 'Variable __name__: ' + __name__ + '\n'
    text += 'Current directory: ' + os.getcwd() + '\n\n'
    text += 'Python path: \n'
    text += '\n'.join(sys.path) + '\n'
    text += '\n## End of diagnostic output ## \n'
    return text

