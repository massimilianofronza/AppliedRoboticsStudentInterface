cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
 src	## to include student_functions.hpp and clipper.hpp
)

## Add executables
add_executable(calibrate
   src/camera_calibration.cpp
)


add_library(ClipperLib SHARED
   src/clipper.cpp
)

## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
   src/student_interface.cpp
   src/student_imageUndistort.cpp
   src/student_extrinsicCalib.cpp
   src/student_findPlaneTransform.cpp
   src/student_unwarp.cpp
   src/student_loadImage.cpp
   src/student_genericImageListener.cpp
   src/student_findRobot.cpp
   src/student_processMap.cpp
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}
  ClipperLib
)

target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}
)

