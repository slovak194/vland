cmake_minimum_required(VERSION 3.5.1)
project(vland)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}, libs: ${OpenCV_LIBRARIES}")
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(modules/cmake_deb_helpers)
set(CMAKE_MODULE_PATH "${cmake_deb_helpers_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
include(installation)
include(deb_target)


add_subdirectory(generated)

add_executable(main_node src/main.cpp)
target_link_libraries(main_node
        mpc_6dof
        opencv_core)


add_install_component(vland)
install(TARGETS main_node DESTINATION /usr/bin COMPONENT vland)

set(DEB_VERSION 1.0)
add_deb_target(vland)
