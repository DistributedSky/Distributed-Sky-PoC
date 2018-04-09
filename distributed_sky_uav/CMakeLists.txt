cmake_minimum_required(VERSION 2.8.3)
project(distributed_sky_uav)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
        message_generation
        geographic_msgs
)

catkin_python_setup()

add_message_files(
    FILES
        RouteConfirmationRequest.msg
)

generate_messages(
    DEPENDENCIES
    geographic_msgs
)

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/route_confirmer_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/route_confirmer.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
