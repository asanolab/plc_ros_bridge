cmake_minimum_required(VERSION 2.8.0)
project(plc_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
)

catkin_python_setup()

catkin_package()


#############
## Install ##
#############

catkin_install_python(
  PROGRAMS
    scripts/plc_ros_bridge_node.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
