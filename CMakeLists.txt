cmake_minimum_required(VERSION 3.9)
project(vland)

add_subdirectory(modules/cmake_deb_helpers)
set(CMAKE_MODULE_PATH "${cmake_deb_helpers_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
include(installation)
include(deb_target)


add_executable(main_node src/main.cpp)

add_install_component(vland)
install(TARGETS main_node DESTINATION /usr/bin COMPONENT vland)

set(DEB_VERSION 1.0)
add_deb_target(vland)
