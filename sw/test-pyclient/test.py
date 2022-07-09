#!/usr/bin/env python3
from pymodbus.client.sync import ModbusTcpClient

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

client = ModbusTcpClient('127.0.0.1', port=4303)
client.connect()

log.debug("Running ReadHoldingRegistersRequest")
rq = ReadHoldingRegistersRequest(0, 1, unit=UNIT)
rr = client.execute(rq)
log.debug(str(rr) + ': ' + str(rr.registers))
