#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import socket
import time


class PLCInterfaceBase(object):
    def __init__(self, host='192.168.0.2', port=8501, buffer_size=1024):
        self.host = host
        self.port = port
        self.client = None
        self.buffer_size = buffer_size
        self.connection_opened = False


    def open(self):
        if not self.connection_opened:
            try:
                self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client.connect((self.host, self.port))
                self.connection_opened = True
                time.sleep(0.5)
                print('Connected to PLC\n')
            except Exception as e:
                print('Connection error')


    def is_connected(self):
        return self.connection_opened


    def close(self):
        self.client.close()
