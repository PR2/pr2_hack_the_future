cmake_minimum_required(VERSION 2.8.3)
project(slider_gui)
find_package(catkin REQUIRED COMPONENTS actionlib rospy pr2_controllers_msgs trajectory_msgs korg_nanokontrol program_queue face_detector sound_play rviz_backdrop sensor_msgs std_srvs pr2_mechanism_msgs)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${Boost_INCLUDE_DIRS})

add_service_files(
  FILES
  RunProgram.srv
)

generate_messages(
    DEPENDENCIES std_msgs
)
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS actionlib rospy pr2_controllers_msgs trajectory_msgs korg_nanokontrol program_queue face_detector sound_play rviz_backdrop sensor_msgs std_srvs pr2_mechanism_msgs 
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY icons/ images/ launch/ nodes/ sounds/ urdf/ worlds/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(PROGRAMS scripts/image_hack.py scripts/setup_env_for_robot.sh scripts/upload-program.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



