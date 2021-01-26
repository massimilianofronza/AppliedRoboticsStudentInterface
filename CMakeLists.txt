cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
 src/include	## custom headers folder
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
   src/student_interface.cpp	## main file
   src/visual/student_extrinsicCalib.cpp
   src/visual/student_findPlaneTransform.cpp
   src/visual/student_findRobot.cpp
   src/visual/student_genericImageListener.cpp
   src/visual/student_imageUndistort.cpp
   src/visual/student_loadImage.cpp
   src/visual/student_processMap.cpp
   src/visual/student_unwarp.cpp
   src/path/dubins_core.cpp
   src/path/dubins_utility.cpp
   src/path/dubins_plot.cpp
   src/path/collision_core.cpp
   src/path/collision_utility.cpp
   src/path/collision_plot.cpp
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}
  ClipperLib
)

target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}
)

