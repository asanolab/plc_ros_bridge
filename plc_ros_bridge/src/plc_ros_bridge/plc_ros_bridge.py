#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from threading import Lock
from .plc_ros_bridge_publisher import Publisher
from .plc_ros_bridge_subscriber import Subscriber


class PLCROSBridge:
    def __init__(self, plc_interface, plc_maker, ip, config, pub_rate):
        self.pi = plc_interface
        self.plc_maker = plc_maker
        self.ip = ip
        self.config = config
        self.pub_rate = pub_rate

        self.timer = None
        self.mutex = Lock()

        self.subscribers = []
        self.publishers = []

        self.load_config(self.config)


    def is_connected(self):
        with self.mutex:
            return self.pi.is_connected()


    def connect_plc(self, ip):
        with self.mutex:
            self.pi.open(ip)
            return self.pi.is_connected()


    def disconnect_plc(self):
        with self.mutex:
            if self.pi.is_connected():
                self.pi.close()


    # read wrapper for each PLC devices
    def read_plc(self, plc_dev, plc_data_type):
        with self.mutex:
            if self.plc_maker == 'Keyence':
                return self.pi.read_plc(plc_dev, plc_data_type)
            # other PLC's will be added here
            else:
                rospy.logerr("%s interface has not implemented yet", self.plc_maker)


    # write wrapper for each PLC devices
    def write_plc(self, plc_dev, plc_data_type, plc_data):
        with self.mutex:
            if self.plc_maker == 'Keyence':
                return self.pi.write_plc(plc_dev, plc_data_type, plc_data)
            # other PLC's will be added here
            else:
                rospy.logerr("%s interface has not implemented yet", self.plc_maker)


    def load_config(self, config):
        # publisher
        rospy.loginfo("Loading Publishers config")
        for cfg in config['Publishers']:
            pub = Publisher(cfg['topic_name'], cfg['topic_type'], cfg['plc_dev'], self)
            self.publishers.append(pub)

        # subscriber
        rospy.loginfo("Loading Subscribers config")
        for cfg in config['Subscribers']:
            sub = Subscriber(cfg['topic_name'], cfg['topic_type'], cfg['plc_dev'], self)
            self.subscribers.append(sub)


    def convert_topic_type(self, topic_type):
        if topic_type == 'Bool':
            plc_data_type = 'BOOL'
        elif topic_type == 'Int16':
            plc_data_type = 'INT16'
        elif topic_type == 'UInt16':
            plc_data_type = 'UINT16'
        elif topic_type == 'Int32':
            plc_data_type = 'INT32'
        elif topic_type == 'UInt32':
            plc_data_type = 'UINT32'
        else:
            rospy.logerr("%s topic_type has not supported", topic_type)

        return plc_data_type


    def run(self):
        # timer enables additional loop with constant period
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.pub_rate), self.update)


    def update(self, event):
        if not self.is_connected():
            rospy.loginfo("Connecting to PLC at {}".format(self.ip))
            self.connect_plc(self.ip)
            rospy.loginfo("Connected to PLC at {}".format(self.ip))
        else:
            for pub in self.publishers:
                pub.publish()
