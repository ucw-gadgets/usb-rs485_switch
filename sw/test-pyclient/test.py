#!/usr/bin/env python3
from pymodbus.client.sync import ModbusTcpClient

from pymodbus.exceptions import *
from pymodbus.diag_message import *
from pymodbus.file_message import *
from pymodbus.other_message import *
from pymodbus.mei_message import *
from pymodbus.register_read_message import *

import sys

import logging
log = logging.getLogger()
logging.basicConfig(level=logging.DEBUG)
logging.getLogger('pymodbus').setLevel(level=logging.INFO)

UNIT = 3

client = ModbusTcpClient('127.0.0.1', port=4300)
client.connect()

log.debug("Reading input registers")
rr = client.read_input_registers(1, 17, unit=UNIT)
assert not rr.isError()
log.debug(rr.registers)

log.debug("Setting holding registers")
rr = client.write_register(1, 1152, unit=UNIT)
assert not rr.isError()

log.debug("Reading holding registers")
rr = client.read_holding_registers(1, 4, unit=UNIT)
assert not rr.isError()
log.debug(rr.registers)
