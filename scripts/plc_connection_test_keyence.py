#!/usr/bin/env python
# -*- coding: utf-8 -*-

from plc_interface.keyence import PLCInterfaceKeyence


plc = PLCInterfaceKeyence('192.168.0.10')
plc.open()
