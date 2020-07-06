project(task1)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(undistortImage undistort_image.cpp)
target_link_libraries(undistortImage ${OpenCV_LIBS})
