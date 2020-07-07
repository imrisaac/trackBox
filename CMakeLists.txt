cmake_minimum_required(VERSION 3.0)

project(TrackBox)

find_package(OpenCV 4.1.0 REQUIRED)
find_package(realsense2 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# include_directories( ${PROJECT_SOURCE_DIR}/modules/jetson-utils)
include_directories(/usr/include/gstreamer-1.0 /usr/lib/aarch64-linux-gnu/gstreamer-1.0/include /usr/include/glib-2.0 /usr/include/libxml2 /usr/lib/aarch64-linux-gnu/glib-2.0/include/)

# add_subdirectory(${PROJECT_SOURCE_DIR}/modules/jetson-utils)

include_directories(${OpenCV_INCLUDE_DIRS} 
                    ${PROJECT_SOURCE_DIR}
                    ${PROJECT_SOURCE_DIR}/trackBox)

add_executable(trackBox src/main.cpp
                        src/trackBox.cpp
                        )

target_link_libraries(trackBox ${OpenCV_LIBS} ${realsense2_LIBRARY})