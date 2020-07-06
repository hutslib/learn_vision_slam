cmake_minimum_required( VERSION 2.8 )
project( disparity )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )
# Pangolin
find_package(Pangolin REQUIRED)
message(STATUS ${Pangolin_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(disparity disparity.cpp)
target_link_libraries(disparity ${Pangolin_LIBRARIES})
target_link_libraries(disparity ${OpenCV_LIBS})
# C++ 11
add_definitions(-std=c++11)
