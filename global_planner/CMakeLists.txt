cmake_minimum_required(VERSION 2.8.3)
project(distributed_sky_uav)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
        geographic_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/global_planner_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/global_planner.launch
        launch/global_planner_infrastructure.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
