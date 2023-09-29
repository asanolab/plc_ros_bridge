#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from plc_ros_bridge.plc_ros_bridge import PLCROSBridge
from plc_interfaces.plc_interface_keyence import PLCInterfaceKeyence

def main():
    rospy.init_node('plc_ros_bridge_node', anonymous=True)

    # Get the parameters
    try:
        ip = rospy.get_param('~ip')
    except:
        rospy.logfatal('Failed to get "ip" parameter')
        exit(1)

    try:
        config = rospy.get_param('~config')
    except:
        rospy.logfatal('Failed to get "config" parameter')
        exit(2)

    pub_rate = rospy.get_param('~pub_rate', 10)
    plc_maker = rospy.get_param('~plc_maker')

    # PLC interface
    if plc_maker == 'Keyence':
        plc_interface = PLCInterfaceKeyence()
    # other PLC's will be added here
    else:
        rospy.logerr('%s interface has not supported yet', plc_maker)

    # Generate the bridge
    bridge = PLCROSBridge(plc_interface, plc_maker, ip, config, pub_rate)

    # Run the bridge
    bridge.run()
    rospy.spin()

    bridge.disconnect_plc()


if __name__ == '__main__':
    main()
