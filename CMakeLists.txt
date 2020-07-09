cmake_minimum_required(VERSION 3.0)

project(TrackBox)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV 4.1.0 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GLIB REQUIRED glib-2.0)

# include_directories( ${PROJECT_SOURCE_DIR}/modules/jetson-utils)
include_directories(/usr/include/gstreamer-1.0 /usr/lib/aarch64-linux-gnu/gstreamer-1.0/include /usr/include/glib-2.0 /usr/include/libxml2 /usr/lib/aarch64-linux-gnu/glib-2.0/include/)

# add_subdirectory(${PROJECT_SOURCE_DIR}/modules/jetson-utils)

include_directories(${OpenCV_INCLUDE_DIRS} 
                    ${PROJECT_SOURCE_DIR}
                    ${GLIB_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/src
                    ${PROJECT_SOURCE_DIR}/src/trackBox
                    ${PROJECT_SOURCE_DIR}/src/target
                    ${PROJECT_SOURCE_DIR}/src/targetTrack)

add_executable(trackBox src/main.cpp
                        src/target/target.cpp
                        src/trackBox/trackBox.cpp
                        src/targetTrack/targetTrack.cpp
                        )

target_link_libraries(trackBox ${OpenCV_LIBS} ${realsense2_LIBRARY} ${GLIB_LDFLAGS})