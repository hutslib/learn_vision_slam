cmake_minimum_required( VERSION 2.8 )
project( geometry )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )
# Pangolin
find_package(Pangolin REQUIRED)
message(STATUS ${Pangolin_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
# 为使用 sophus，您需要使用find_package命令找到它
find_package(Sophus REQUIRED)
message(STATUS ${Sophus_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries(draw_trajectory ${Sophus_LIBRARIES} )
target_link_libraries(draw_trajectory ${Pangolin_LIBRARIES})
# C++ 11
add_definitions(-std=c++11)
