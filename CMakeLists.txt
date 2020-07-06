cmake_minimum_required(VERSION 3.10)
project(odomcpp)
add_definitions(-std=c++14)


find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )


add_library(${PROJECT_NAME} src/main.cpp src/camera.cpp src/visual_odometery.cpp) # optional
	
include(CMakeToolsHelpers OPTIONAL)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${EIGEN3_INCLUDE_DIR} )

include_directories( ${CMAKE_SOURCE_DIR}/include)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

add_executable(visodom src/main.cpp src/camera.cpp src/visual_odometery.cpp)

target_link_libraries( visodom ${OpenCV_LIBS} ${EIGEN3})
