cmake_minimum_required(VERSION 2.8.3)
project(queue_web)
find_package(catkin REQUIRED )

catkin_package(
    DEPENDS # TODO add dependencies
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(FILES install.sh queue-web DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY new_www/ www/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
