#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from plc_interfaces.plc_interface_keyence import PLCInterfaceKeyence

plc = PLCInterfaceKeyence()
plc.open('192.168.0.10')
