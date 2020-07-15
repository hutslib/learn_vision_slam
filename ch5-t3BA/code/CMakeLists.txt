cmake_minimum_required( VERSION 2.8 )
project( vo1 )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

FIND_PACKAGE(Sophus REQUIRED)

include_directories(
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable( GN-BA GN-BA.cpp  )
target_link_libraries( GN-BA ${Sophus_LIBRARIES} )
