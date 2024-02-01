#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import struct
from plc_interface_base import PLCInterfaceBase


class PLCInterfaceMitsubishi(PLCInterfaceBase):
    def __init__(self, host='192.168.0.2', port=1025, buffer_size=1024):
        super(PLCInterfaceMitsubishi, self).__init__(host, port, buffer_size)


    def make_SLMP_3E_frame_binary_common(self):
        sub_header  = b'\x50' + b'\x00'  # 5000 (固定)
        network_id  = b'\x00'            # 00 (自局)
        station_id  = b'\xFF'            # FF (自局)
        unit_io     = b'\xFF' + b'\x03'  # 03FF (CPUユニット)
        multi_drop  = b'\x00'            # 00 (マルチドロップでない)

        common = sub_header + network_id + station_id + unit_io + multi_drop

        return common


    def make_SLMP_3E_frame_binary(self, data_length, timer, cmd, sub_cmd, device, device_num, write_data=None):
        common = self.make_SLMP_3E_frame_binary_common()
        device_code, device_id = self.split_device(device)

        if write_data: # write
            msg = common + data_length + timer + cmd + sub_cmd + device_id + device_code + device_num + write_data
        else: # read
            msg = common + data_length + timer + cmd + sub_cmd + device_id + device_code + device_num

        return msg


    def make_SLMP_3E_frame_binary_read_bit (self, device):
        # Request command (read bit)
        data_length = b'\x0C' + b'\x00'  # 000C (12 byte)
        timer       = b'\x00' + b'\x00'  # 0000 (無限待ち)
        cmd         = b'\x01' + b'\x04'  # 0401 (一括読み出し)
        sub_cmd     = b'\x01' + b'\x00'  # 0001 (ビットデバイス1ビット読み出し)
        device_num  = b'\x01' + b'\x00'  # 0001 (1点)

        msg = self.make_SLMP_3E_frame_binary(data_length, timer, cmd, sub_cmd, device, device_num)

        return msg


    def make_SLMP_3E_frame_binary_read_word (self, device):
        # Request command (read word)
        data_length = b'\x0C' + b'\x00'  # 000C (12 byte)
        timer       = b'\x00' + b'\x00'  # 0000 (無限待ち)
        cmd         = b'\x01' + b'\x04'  # 0401 (一括読み出し)
        sub_cmd     = b'\x00' + b'\x00'  # 0000 (ワードデバイス1ワード読み出し)
        device_num  = b'\x01' + b'\x00'  # 0001 (1点)

        msg = self.make_SLMP_3E_frame_binary(data_length, timer, cmd, sub_cmd, device, device_num)

        return msg


    def make_SLMP_3E_frame_binary_write_bit(self, device, write_data):
        # Request command (write bit)
        data_length = b'\x0D' + b'\x00'  # 000D (13 byte)
        timer       = b'\x00' + b'\x00'  # 0000 (無限待ち)
        cmd         = b'\x01' + b'\x14'  # 1401 (一括書き出し)
        sub_cmd     = b'\x01' + b'\x00'  # 0001 (ビットデバイス1点書き込み)
        device_num  = b'\x01' + b'\x00'  # 0001 (1点)

        msg = self.make_SLMP_3E_frame_binary(data_length, timer, cmd, sub_cmd, device, device_num, write_data)

        return msg


    def make_SLMP_3E_frame_binary_write_word(self, device, write_data):
        # Request command (write word)
        data_length = b'\x0D' + b'\x00'  # 000D (13 byte)
        timer       = b'\x00' + b'\x00'  # 0000 (無限待ち)
        cmd         = b'\x01' + b'\x14'  # 1401 (一括書き出し)
        sub_cmd     = b'\x00' + b'\x00'  # 0000 (ワードデバイス1ワード書き込み)
        device_num  = b'\x01' + b'\x00'  # 0001 (1点)

        msg = self.make_SLMP_3E_frame_binary(data_length, timer, cmd, sub_cmd, device, device_num, write_data)

        return msg


    def split_device (self, device):
        # split device into code and id
        device_code = re.sub('[^A-Z]', '', device)
        device_id = re.sub(device_code, '', device)

        if(device_code == 'X'):     # 入力
            device_code = b'\x9C'
        elif(device_code == 'Y'):   # 出力
            device_code = b'\x9D'
        elif(device_code == 'M'):   # 内部リレー
            device_code = b'\x90'
        elif (device_code == 'D'):  # データレジスタ
            device_code = b'\xA8'
        else:
            print('unknown device\n')

        device_id = struct.pack('<H', int(device_id)) + b'\x00'  # 3 bytes.

        return device_code, device_id


    # base command for sending msg to PLC
    def send (self, msg):
        self.client.send(msg)
        res = self.client.recv(self.buffer_size)

        return res


    # read functions
    def read_bit (self, device, debug=False):
        msg = self.make_SLMP_3E_frame_binary_read_bit(device)
        if debug:
            self.print_msg_read(msg)

        res = self.send(msg)

        return bool(self.extract_data_from_response(res, 'bit', debug))


    def read_word (self, device, debug=False):
        msg = self.make_SLMP_3E_frame_binary_read_word(device)
        if debug:
            self.print_msg_read(msg)

        res = self.send(msg)

        return self.extract_data_from_response(res, 'word', debug)


    def read_plc (self, device, data_type, debug=False):
        if data_type == 'BOOL':
            return self.read_bit(device, debug)
        elif data_type == 'INT16':
            return self.read_word(device, debug)
        elif data_type == 'UINT16':
            return self.read_word(device, debug)
        elif data_type == 'INT32':
            return self.read_word(device, debug)
        elif data_type == 'UINT32':
            return self.read_word(device, debug)
        else:
            return self.read_word(device, debug)



    def extract_data_from_response(self, res, device_type, debug=False):
        # args
        # - device_type: select bit or word.
        #
        res_hex = []
        for i in range(len(res)):
            res_hex.append(hex(ord(res[i]))[2:])  # remove 0x

        sub_header  = res_hex[0] + ' ' + res_hex[1]
        network_id  = res_hex[2]
        station_id  = res_hex[3]
        unit_io     = res_hex[4] + ' ' + res_hex[5]
        multi_drop  = res_hex[6]
        data_length = res_hex[7] + ' ' + res_hex[8]
        data_length_int_str = str(int(res_hex[8], 16)) + str(int(res_hex[7], 16)) # convert endian to (L H) and connect data as string
        data_length_int = int(data_length_int_str)
        end_code     = res_hex[9] + ' ' + res_hex[10]
        end_code_mod = res_hex[10] + res_hex[9]
        data         = res_hex[11:]
        data_length  = data_length_int - 2  # data_res_length = data_length (x bits) - end_code (2 bits)
        if data_length != len(data):
            print('error')

        # adjustment for endian
        data_rev = []
        if (data_length == 1):
            data_rev = data
        else:
            for i in range(data_length-1):
                data_rev = data[::-1]  # reverse for endian

        # extract value
        data_rev_str = ''
        if data == []:
            data_value = None
        else:
            for i in range(data_length):
                data_rev_str += str(data_rev[i])
            if (device_type == 'bit'):
                data_value = int(data_rev_str[0])  # extract the fitst bit and convert str to int
            elif (device_type == 'word'):
                data_value = int(data_rev_str, 16)
            else:
                print('choose bit or word')

        if debug:
            print("Response:")
            print('all:', repr(res))
            print('sub header :', sub_header)
            print('network_id :', network_id)
            print('station_id :', station_id)
            print('unit_io    :', unit_io)
            print('multi_drop :', multi_drop)
            print('data_length:', data_length, '->', data_length_int, 'bits')
            print('end_code   :', end_code,    '->', end_code_mod)
            print('data       :', data ,       '->', data_value)
            print('---------------------------------------\n')

        return data_value


    # write functions
    def write_bit(self, device, data, debug=True):
        data_01 = int(data)  # convert True/False -> 1/0
        write_data_bits = str(data_01) + '0'  # add 0 to make '10' or '00'
        write_data = write_data_bits.decode('hex')  #16進数文字列 + byteに変換

        msg = self.make_SLMP_3E_frame_binary_write_bit(device, write_data)
        if debug:
            self.print_msg_write(msg)

        return self.send(msg)


    def write_word(self, device, data, debug=False):
        write_data = data

        msg = self.make_SLMP_3E_frame_binary_write_word(device, write_data)
        if debug:
            self.print_msg_write(msg)

        return self.send(msg)


    def write_plc (self, device, data_type, data, debug=False):
        if data_type == 'BOOL':
            return self.write_bit(device, data, debug)
        else:
            return self.write_word(device, data, debug)


    # functions for msg check and print
    def print_msg_write(self, msg):
        req_hex = []
        for i in range(22):
            req_hex.append(hex(ord(msg[i]))[2:])  # remove 0x

        device_id_req_int_str = str(int(req_hex[17], 16)) + str(int(req_hex[16], 16)) + str(int(req_hex[15], 16)) # reverse endian and connect data as string
        device_id_req_int = int(device_id_req_int_str)

        print('---------------------------------------')
        print('Request (write):')
        print('all:', repr(msg))
        print('sub header :', req_hex[0], req_hex[1])
        print('network_id :', req_hex[2])
        print('station_id :', req_hex[3])
        print('unit_io    :', req_hex[4], req_hex[5])
        print('multi_drop :', req_hex[6])
        print('data_length:', req_hex[7], req_hex[8])
        print('timer      :', req_hex[9], req_hex[10])
        print('command    :', req_hex[11], req_hex[12])
        print('sub command:', req_hex[13], req_hex[14])
        print('device_id  :', req_hex[15], req_hex[16], req_hex[17], '->',  device_id_req_int)
        print('device_code:', req_hex[18])
        print('device_num :', req_hex[19], req_hex[20])
        print('write_data :', req_hex[21])
        print('')

        # Data processing:
        # - get character code of each byte by ord()
        # - display the number with 16-digits expression by hex()


    def print_msg_read(self, msg):
        req_hex = []
        for i in range(21):
            req_hex.append(hex(ord(msg[i]))[2:])  # remove 0x

        device_id_req_int_str = str(int(req_hex[17], 16)) + str(int(req_hex[16], 16)) + str(int(req_hex[15], 16)) # reverse endian and connect data as string
        device_id_req_int = int(device_id_req_int_str)

        print('---------------------------------------')
        print('Request (read):')
        print('all:', repr(msg))
        print('sub header :', req_hex[0], req_hex[1])
        print('network_id :', req_hex[2])
        print('station_id :', req_hex[3])
        print('unit_io    :', req_hex[4], req_hex[5])
        print('multi_drop :', req_hex[6])
        print('data_length:', req_hex[7], req_hex[8])
        print('timer      :', req_hex[9], req_hex[10])
        print('command    :', req_hex[11], req_hex[12])
        print('sub command:', req_hex[13], req_hex[14])
        print('device_id  :', req_hex[15], req_hex[16], req_hex[17], '->',  device_id_req_int)
        print('device_code:', req_hex[18])
        print('device_num :', req_hex[19], req_hex[20])
        print('')

        # Data processing:
        # - get character code of each byte by ord()
        # - display the number with 16-digits expression by hex()
