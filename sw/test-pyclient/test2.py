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

UNIT = 42

client = ModbusTcpClient('127.0.0.1', port=4306)
client.connect()

log.debug("Reading holding registers")
rr = client.read_holding_registers(0, 1, unit=UNIT)
assert not rr.isError()
log.debug(rr.registers)
