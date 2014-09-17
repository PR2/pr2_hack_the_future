cmake_minimum_required(VERSION 2.8.3)
project(hack_the_web_program_executor)
find_package(catkin REQUIRED)
catkin_package(
    DEPENDS # TODO add dependencies
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(PROGRAMS scripts/poser_real_robot.py scripts/robot_program_executor.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
