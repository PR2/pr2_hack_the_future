# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(program_queue)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs std_srvs hack_the_web_program_executor message_generation)
#uncomment if you have defined messages
add_message_files(
  FILES
  Output.msg
  ProgramInfo.msg
  Program.msg 
)

add_service_files(
  FILES
CallProgram.srv    
CreateUser.srv      
GetOutput.srv   
 GetQueue.srv 
 QueueProgram.srv
ClearQueue.srv    
 DequeueProgram.srv  
GetProgram.srv  
 Login.srv     
RunProgram.srv
CreateProgram.srv 
 GetMyPrograms.srv  
 GetPrograms.srv  
Logout.srv    
UpdateProgram.srv
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS rospy std_msgs std_srvs hack_the_web_program_executor python-bcrypt
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


#Add install files 
install(PROGRAMS nodes/program_queue_mysql.py nodes/program_queue.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY nodes/ msg/ launch/ srv/ test/ backups/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})






