#!/usr/bin/env python3
# A simple utility for controlling the USB-RS485 Switch

import argparse
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse, ModbusExceptions
import sys

# FIXME
#import logging
#log = logging.getLogger()
#logging.basicConfig(level=logging.DEBUG)
#logging.getLogger('pymodbus').setLevel(level=logging.INFO)

parser = argparse.ArgumentParser(description='Manage the USB-RS485 switch')
parser.add_argument('-p', '--port', type=int, default=4300, help='TCP port of the management bus (default: 4300)')

args = parser.parse_args()

modbus = ModbusTcpClient('127.0.0.1', port=args.port)
modbus.connect()

def show_port_config():
    print('Port  Rate   Parity  Power  Timeout [ms]')
    for port in range(1, 9):
        rr = modbus.read_holding_registers(1, 4, unit=port)
        check_modbus_error(rr)
        baud, parity, powered, timeout = rr.registers

        if parity == 0:
            par = 'none'
        elif parity == 1:
            par = 'odd'
        elif parity == 2:
            par = 'even'
        else:
            par = '???'

        if powered > 0:
            pow = 'on'
        else:
            pow = 'off'

        print(f'{port}    {baud*100:6}  {par:4}    {pow:3}   {timeout:5}')

def check_modbus_error(resp):
    if resp.isError():
        assert isinstance(resp, ExceptionResponse)
        print(f'Management bus error: {ModbusExceptions.decode(resp.exception_code)}', file=sys.stderr)
        sys.exit(1)

show_port_config()
