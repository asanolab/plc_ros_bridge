#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import time

# 上位リンク通信のコマンド(see manual "8-4 コマンド一覧")

# general description
# msg(read)  = cmd + " " + device_type + device_num + data_format + separator
#  ex ->     = WR  + " " + R           + 1000       + .U          + " " + 0/1    + separator
# msg(write) = cmd + " " + device_type + device_num + data_format + " " + data   + separator
#  ex ->     = RD  + " " + R           + 1000       + .U          + separator

# cmd
# - ?K: 機種の問い合わせ
# - RD: データ読み込み
# - WR: データ書き込み

# device_type + device_num
# depend on device

# data_format
# - "": 指定なし
# - .U: 16bit unsigned
# - .S: 16bit signed
# - .D: 32bit unsigned
# - .L: 32bit signed
# - .H: 16bit hex

# separator
# - '\r'  区切り符号CR

# port
# - 8501 (default)

class PLCInterfaceKeyence(object):
    def __init__(self, host='localhost', port=8501, buffer_size=1024):
        self.host = host
        self.port = port
        self.client = None
        self.buffer_size = buffer_size
        self.connection_opened = False

    def open(self, ip):
        if not self.connection_opened:
            try:
                self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client.connect((ip, self.port))
                self.connection_opened = True
                time.sleep(0.5)
                print 'Connected to {}'.format(self.device_check())
            except Exception as e:
                print 'Connection error'

    def is_connected(self):
        return self.connection_opened

    def close(self):
        self.client.close()

    # base command for sending msg to PLC
    def send (self, msg):
        self.client.send(msg.encode('utf-8'))
        response = self.client.recv(self.buffer_size).decode('utf-8')
        # format:
        # - read : '00000\r\n'
        # - write: 'OK\r\n'
        # print "Send: " + msg
        # print "Recieved: " + response
        return response

    # read functions
    def read (self, device):
        msg_read = 'RD ' + device + '' + '\r'
        return self.send(msg_read)

    def read_bool (self, device):
        msg_read = 'RD ' + device + '' + '\r'
        return int(self.send(msg_read))  # cast int is needed

    def read_uint16 (self, device):
        msg_read = 'RD ' + device + '.U' + '\r'
        return self.send(msg_read)

    def read_int16 (self, device):
        msg_read = 'RD ' + device + '.S' + '\r'
        return self.send(msg_read)

    def read_uint32 (self, device):
        msg_read = 'RD ' + device + '.D' + '\r'
        return self.send(msg_read)

    def read_int32 (self, device):
        msg_read = 'RD ' + device + '.L' + '\r'
        return self.send(msg_read)

    def read_hex16 (self, device):
        msg_read = 'RD ' + device + '.H' + '\r'
        return self.send(msg_read)

    def read_plc (self, device, data_type):
        if data_type == 'BOOL':
            return bool(self.read_bool(device))
        elif data_type == 'INT16':
            return int(self.read_int16(device))
        elif data_type == 'UINT16':
            return int(self.read_uint16(device))
        elif data_type == 'INT32':
            return int(self.read_int32(device))
        elif data_type == 'UINT32':
            return int(self.read_uint32(device))
        else:
            return self.read(device)

    # write functions
    def write (self, device, data):
        msg_write = 'WR ' + device + ' ' + str(data) + '\r'
        return self.send(msg_write)

    def write_bool (self, device, data):
        data_01 = int(data)  # convert True/False -> 1/0
        msg_write = 'WR ' + device + ' ' + str(data_01) + '\r'
        return self.send(msg_write)

    def write_plc (self, device, data_type, data):
        if data_type == 'BOOL':
            return self.write_bool(device, data)
        else:
            return self.write(device, data)

    # other functions
    def device_check (self):
        res = int(self.send('?K\r\n'))
        if res == 55:
            dev = 'KV-7500'
        elif res == 128:
            dev = 'KV-NC32T'
        else:
            dev = 'Unknown device'

        return dev
