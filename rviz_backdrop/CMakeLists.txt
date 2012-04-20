cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

set(SOURCE_FILES
  src/backdrop_display.cpp
)

## Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_backdrop", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above.
##
## Although this puts "rviz_backdrop" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_backdrop.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

