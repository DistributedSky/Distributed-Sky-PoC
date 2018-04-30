cmake_minimum_required(VERSION 2.8.3)
project(web_visualiser)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
        geographic_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/web_visualiser_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/web_visualiser.launch
        launch/web_visualiser_infrastructure.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
