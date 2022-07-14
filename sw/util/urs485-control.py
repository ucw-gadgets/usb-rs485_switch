#!/usr/bin/env python3
# A simple utility for controlling the USB-RS485 Switch

import argparse
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.pdu import ExceptionResponse, ModbusExceptions
import sys

parity_by_name = {'none': 0, 'odd': 1, 'even': 2}
parity_by_number = {y: x for x, y in parity_by_name.items()}


def cmd_config(args):
    if (args.baud is not None or
        args.parity is not None or
        args.power is not None or
        args.timeout is not None):
        return cmd_config_set(args)

    ports = parse_port_list(args.p, True)

    print('Port  Baud   Parity  Power  Timeout [ms]')
    for port in ports:
        rr = modbus.read_holding_registers(1, 4, unit=port)
        check_modbus_error(rr)
        baud, parity, powered, timeout = rr.registers

        if powered > 0:
            pow = 'on'
        else:
            pow = 'off'

        print(f'{port}    {baud*100:6}  {parity_by_number[parity]:4}    {pow:3}   {timeout:5}')


def cmd_config_set(args):
    if not args.baud:
        baud = None
    elif args.baud not in range(1200, 115201):
        die('Baud rate out of range')
    elif args.baud % 100 != 0:
        die('Baud rate must be divisible by 100')
    else:
        baud = args.baud // 100

    if args.parity is None:
        parity = None
    elif args.parity in parity_by_name:
        parity = parity_by_name[args.parity]
    else:
        die(f'Invalid parity mode {args.parity}')

    if args.power not in [None, 0, 1]:
        die(f'Invalid power mode {args.power}')

    if not(args.timeout is None or args.timeout in range(1, 65536)):
        die('Timeout out of range')

    for port in parse_port_list(args.p, False):
        if baud is not None:
            rr = modbus.write_register(1, baud, unit=port)
            check_modbus_error(rr)
        if parity is not None:
            rr = modbus.write_register(2, parity, unit=port)
            check_modbus_error(rr)
        if args.power is not None:
            rr = modbus.write_register(3, args.power, unit=port)
            check_modbus_error(rr)
        if args.timeout is not None:
            rr = modbus.write_register(4, args.timeout, unit=port)
            check_modbus_error(rr)


def cmd_status(args):
    ...


def parse_port_list(ports, default_all):
    try:
        if ports == "" or ports is None:
            if default_all:
                return list(range(1, 9))
            else:
                die("For setting parameters, you need to select ports explicitly")
        elif ports == "all":
            return list(range(1, 9))
        else:
            out = set()
            for a in ports.split(','):
                rng = [int(x) for x in a.split('-')]
                if not all(p in range(1, 9) for p in rng):
                    die('Ports are numbered between 1 and 8')
                if len(rng) == 1:
                    out.add(rng[0])
                elif len(rng) == 2:
                    if rng[0] > rng[1]:
                        raise ValueError
                    out = out | set(range(rng[0], rng[1] + 1))
                else:
                    raise ValueError
            return sorted(list(out))
    except ValueError:
        die(f'Cannot parse port specification: {ports}')


def check_modbus_error(resp):
    if resp.isError():
        assert isinstance(resp, ExceptionResponse)
        die(f'Management bus error: {ModbusExceptions.decode(resp.exception_code)}')


def die(msg):
    print(msg, file=sys.stderr)
    sys.exit(1)


parser = argparse.ArgumentParser(description='Manage the USB-RS485 switch')
parser.add_argument('-i', '--ip', default='127.0.0.1', help='IP address of the switch (default: 127.0.0.1)')
parser.add_argument('-p', '--port', type=int, default=4300, help='TCP port of the management bus (default: 4300)')
sub = parser.add_subparsers(dest='command', metavar='command', required=True)

p_config = sub.add_parser('config', help='show/set switch configuration')
p_config.add_argument('-p', help='on which ports to act (e.g., "3,5-7" or "all")')
p_config.add_argument('--baud', type=int, help='baud rate (1200-115200)')
p_config.add_argument('--parity', help='parity (none/odd/even)')
p_config.add_argument('--power', type=int, help='deliver power to the port (0/1)')
p_config.add_argument('--timeout', type=int, help='reply timeout [ms]')

p_status = sub.add_parser('status', help='show port status')

args = parser.parse_args()

modbus = ModbusTcpClient(args.ip, port=args.port)
modbus.connect()

cmd = args.command
if cmd == 'config':
    cmd_config(args)
elif cmd == 'status':
    cmd_status(args)
