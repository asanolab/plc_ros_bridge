#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import *


class Subscriber:
    def __init__(self, topic_name, topic_type, plc_dev, bridge):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.plc_dev = plc_dev
        self.bridge = bridge

        self.plc_data_type = bridge.convert_topic_type(topic_type)
        rospy.Subscriber(self.topic_name, eval(self.topic_type), self.callback, queue_size=1)


    def callback(self, msg):
        self.bridge.write_plc(self.plc_dev, self.plc_data_type, msg.data)
