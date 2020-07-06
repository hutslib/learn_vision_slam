cmake_minimum_required( VERSION 2.8 )
project( disparity )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )
# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries(gaussnewton ${OpenCV_LIBS})
# C++ 11
add_definitions(-std=c++11)
