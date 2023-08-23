#!/usr/bin/env python3
# A simple utility for controlling the USB-RS485 Switch

import argparse
import logging
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ConnectionException
from pymodbus.mei_message import *
from pymodbus.pdu import ExceptionResponse, ModbusExceptions
import re
import sys

parity_by_name = {'none': 0, 'odd': 1, 'even': 2}
parity_by_number = {y: x for x, y in parity_by_name.items()}

power_by_number = {0: 'off', 1: 'on'}


def cmd_config(args):
    if (args.baud is not None or
        args.parity is not None or
        args.power is not None or
        args.timeout is not None or
        args.description is not None):
        return cmd_config_set(args)

    ports = parse_port_list(args.p, True)

    print('Port  Descr.    Baud   Parity  Power  Timeout [ms]')
    for port in ports:
        rr = modbus.read_holding_registers(1, 8, slave=port)
        check_modbus_error(rr)
        baud, parity, powered, timeout = rr.registers[:4]
        descr = [chr(rr.registers[4+i] >> 8) + chr(rr.registers[4+i] & 0x7f) for i in range(4)]
        print(f'{port}     {"".join(descr)} {baud*100:6}  {parity_by_number[parity]:4}    {power_by_number[powered]:3}   {timeout:5}')


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

    if args.description is not None:
        if len(args.description) > 8:
            die('Description may have at most 8 characters')
        descr = f'{args.description:8}'.encode('US-ASCII')
    else:
        descr = None

    for port in parse_port_list(args.p, False):
        if baud is not None:
            rr = modbus.write_register(1, baud, slave=port)
            check_modbus_error(rr)
        if parity is not None:
            rr = modbus.write_register(2, parity, slave=port)
            check_modbus_error(rr)
        if args.power is not None:
            rr = modbus.write_register(3, args.power, slave=port)
            check_modbus_error(rr)
        if args.timeout is not None:
            rr = modbus.write_register(4, args.timeout, slave=port)
            check_modbus_error(rr)
        if descr is not None:
            regs = [(descr[2*i] << 8) + descr[2*i + 1] for i in range(4)]
            rr = modbus.write_registers(5, regs, slave=port)
            check_modbus_error(rr)


def cmd_status(args):
    if args.reset:
        return cmd_status_reset(args)

    ports = parse_port_list(args.p, True)

    row_headings = [
        'Description',
        'Baud rate',
        'Parity',
        'Powered',
        'Timeout [ms]',
        'Current sense',
        'Broadcasts OK',
        'Unicasts OK',
        'Framing errors',
        'Oversize',
        'Undersize',
        'CRC errors',
        'Mismatched',
        'Timeouts',
    ]

    table = []

    for port in ports:
        def u16(i):
            return regs[i-1]

        def u32(i):
            return (regs[i] << 16) + regs[i-1]

        rr = modbus.read_holding_registers(1, 8, slave=port)
        check_modbus_error(rr)
        regs = rr.registers

        desc = "".join([chr(regs[4+i] >> 8) + chr(regs[4+i] & 0x7f) for i in range(4)])
        desc = re.sub(r' +$', "", desc)

        out = [
            desc,
            u16(1) * 100,
            parity_by_number[u16(2)],
            power_by_number[u16(3)],
            u16(4),
        ]

        rr = modbus.read_input_registers(1, 17, slave=port)
        check_modbus_error(rr)
        regs = rr.registers

        out.extend([
            u16(1),
            u32(2),
            u32(4),
            u32(6),
            u32(8),
            u32(10),
            u32(12),
            u32(14),
            u32(16),
        ])

        table.append(out)

    print(f'{"":15}', end="")
    for p in ports:
        col = f'Port {p}'
        print(f'{col:>10}', end="")
    print()

    for r in range(len(row_headings)):
        print(f'{row_headings[r]:15}', end="")
        for i in range(len(ports)):
            print(f'{table[i][r]:>10}', end="")
        print()


def cmd_status_reset(args):
    for port in parse_port_list(args.p, True):
        rr = modbus.write_register(0x1000, 0xdead, slave=port)
        check_modbus_error(rr)


def cmd_version(args):
    fields = [
        ('Vendor',              0 ),
        ('Product',             1 ),
        ('Daemon version',      2 ),
        ('Switch name',         0x80 ),
        ('Serial number',       0x81 ),
        ('Hardware version',    0x82 ),
    ]

    for label, id in fields:
        rq = ReadDeviceInformationRequest(4, id, slave=1)
        rr = modbus.execute(rq)
        check_modbus_error(rr)
        val = rr.information.get(id, b'-').decode('utf-8')
        print(f'{label+":":20} {val}')


def cmd_scan(args):
    if args.dev is not None:
        min, max = args.dev, args.dev
    else:
        min, max = args.min, args.max

    if not(min in range(1, 248) and max in range(1, 248)):
        die("Device addresses must lie between 1 and 247")
    if args.p not in range(0, 9):
        die("Ports are numbered between 0 and 8")

    # XXX: Timeout must be significantly greater than bus timeout in the switch
    bus = ModbusTcpClient(args.ip, port=args.port + args.p, timeout=60)
    bus.connect()

    for addr in range(min, max+1):
        scan_device(bus, addr)


id_field_names = {
    0: 'Vendor name',
    1: 'Product code',
    2: 'Revision',
    3: 'Vendor URL',
    4: 'Product name',
    5: 'Model name',
    6: 'Application name',
}


def scan_device(bus, addr):
    print(f"Probing address {addr}: ", end="")
    sys.stdout.flush()

    rr = bus.read_holding_registers(0, 1, slave=addr)
    if rr.isError():
        if rr.exception_code == ModbusExceptions.GatewayNoResponse:
            print('no response')
            return
        if rr.exception_code == ModbusExceptions.GatewayPathUnavailable:
            print('no path')
            return
    print('responding')

    rq = ReadDeviceInformationRequest(4, 0, slave=addr)
    rr = bus.execute(rq)
    if rr.isError():
        print(f'\tIdentification not supported: {ModbusExceptions.decode(rr.exception_code)}')
        return

    supports_streaming = (rr.conformity & 0x80) != 0
    level = rr.conformity & 0x7f
    ranges = [(1, 0, 2)]
    if level >= 2:
        ranges.append((2, 3, 6))
    if level >= 3:
        ranges.append((3, 0x80, 0xff))
    if level == 0 or level >= 4:
        print(f'\tUnsupported identification conformity level {rr.conformity:02x}')
    else:
        print(f'\t{"Ident. level":20}{level}, {"streaming" if supports_streaming else "non-streaming"}')

    def show_fields(rr):
        for id, raw_val in sorted(rr.information.items()):
            field = id_field_names.get(id, f'Field #{id}')
            try:
                val = raw_val.decode('utf-8')
            except UnicodeDecodeError:
                val = '<cannot decode>'
            print(f'\t{field:20}{val}')

    if supports_streaming:
        for typ, min_id, max_id in ranges:
            id = min_id
            while id >= 0:
                rq = ReadDeviceInformationRequest(typ, id, slave=addr)
                rr = bus.execute(rq)
                if rr.isError():
                    break
                show_fields(rr)
                if rr.more_follows == 0xff:
                    id = rr.next_object_id
                else:
                    id = -1
    else:
        for typ, min_id, max_id in ranges:
            for id in range(min_id, max_id + 1):
                rq = ReadDeviceInformationRequest(4, id, slave=addr)
                rr = bus.execute(rq)
                if not rr.isError():
                    show_fields(rr)


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
        if isinstance(resp, ExceptionResponse):
            die(f'Management bus error: {ModbusExceptions.decode(resp.exception_code)}')
        else:
            die(f'Management bus error: {resp}')


def die(msg):
    print(msg, file=sys.stderr)
    sys.exit(1)


parser = argparse.ArgumentParser(description='Manage the USB-RS485 switch')
parser.add_argument('-i', '--ip', default='127.0.0.1', help='IP address of the switch (default: 127.0.0.1)')
parser.add_argument('-p', '--port', type=int, default=4300, help='TCP port of the management bus (default: 4300)')
parser.add_argument('--debug', default=False, action='store_true', help='show debugging messages of MODBUS library')
sub = parser.add_subparsers(dest='command', metavar='command', required=True)

p_config = sub.add_parser('config', help='show/set switch configuration')
p_config.add_argument('-p', help='on which ports to act (e.g., "3,5-7" or "all")')
p_config.add_argument('--baud', type=int, help='baud rate (1200-115200)')
p_config.add_argument('--parity', help='parity (none/odd/even)')
p_config.add_argument('--power', type=int, help='deliver power to the port (0/1)')
p_config.add_argument('--timeout', type=int, help='reply timeout [ms]')
p_config.add_argument('--description', type=str, help='port description (up to 8 characters)')

p_status = sub.add_parser('status', help='show port status')
p_status.add_argument('-p', help='on which ports to act (e.g., "3,5-7" or "all")')
p_status.add_argument('--reset', default=False, action='store_true', help='reset statistics')

p_version = sub.add_parser('version', help='show switch version')

p_scan = sub.add_parser('scan', help='scan devices on a bus')
p_scan.add_argument('-p', metavar='PORT', type=int, required=True, help='switch port to scan (0 for control port)')
p_scan.add_argument('--dev', metavar='ADDR', type=int, help='device address to scan (default: min to max)')
p_scan.add_argument('--min', metavar='ADDR', type=int, default=1, help='minimum address to scan (default: 1)')
p_scan.add_argument('--max', metavar='ADDR', type=int, default=247, help='maximum address to scan (default: 247)')

args = parser.parse_args()

if args.debug:
    logging.basicConfig(level=logging.DEBUG)
    logging.getLogger('pymodbus').setLevel(level=logging.DEBUG)

try:
    modbus = ModbusTcpClient(args.ip, port=args.port)
    modbus.connect()

    cmd = args.command
    if cmd == 'config':
        cmd_config(args)
    elif cmd == 'status':
        cmd_status(args)
    elif cmd == 'version':
        cmd_version(args)
    elif cmd == 'scan':
        cmd_scan(args)

except ConnectionException as e:
    die(str(e))
