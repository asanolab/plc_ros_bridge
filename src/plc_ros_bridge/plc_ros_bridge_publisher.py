#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import *


class Publisher:
    def __init__(self, topic_name, topic_type, plc_dev, bridge):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.plc_dev = plc_dev
        self.bridge = bridge

        self.plc_data_type = bridge.convert_topic_type(topic_type)
        self.pub = rospy.Publisher(self.topic_name, eval(self.topic_type), queue_size=1)


    def publish(self):
        plc_data = self.bridge.read_plc(self.plc_dev, self.plc_data_type)

        # generate ROS msg
        msg_type = eval(self.topic_type)
        msg = msg_type(data=plc_data)

        self.pub.publish(msg)
