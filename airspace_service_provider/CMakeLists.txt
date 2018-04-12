cmake_minimum_required(VERSION 2.8.3)
project(airspace_service_provider)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
)

catkin_python_setup()


catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/airspace_service_provider_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/airspace_service_provider.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
