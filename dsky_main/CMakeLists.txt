cmake_minimum_required(VERSION 2.8.3)
project(dsky_main)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
)

catkin_python_setup()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/message_to_bag_to_ipfs_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/test.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
