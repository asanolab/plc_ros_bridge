#!/usr/bin/env python
# -*- coding: utf-8 -*-

from plc_interface.mitsubishi import PLCInterfaceMitsubishi


plc = PLCInterfaceMitsubishi('192.168.0.15')
plc.open()
