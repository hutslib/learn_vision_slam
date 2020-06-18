cmake_minimum_required(VERSION 3.5)
SET(CMAKE_BUILD_TYPE "Release")
PROJECT (HelloSLAM)
# include head file
include_directories(include)
# used to build lib
add_library(hello SHARED src/hello.cpp)
# build executable file
add_executable(sayhello useHello.cpp)
# link lib to exectable file
target_link_libraries(sayhello hello)
#install
INSTALL(TARGETS hello LIBRARY DESTINATION /usr/local/lib)
INSTALL(FILES ${PROJECT_SOURCE_DIR}/include/hello.h DESTINATION /usr/local/include)
