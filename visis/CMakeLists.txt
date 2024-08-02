cmake_minimum_required(VERSION 3.5)
project(visis)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(libs)

set(Visis_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(Visis_INCLUDE_DIR ${Visis_ROOT}/include)
set(Visis_SRC_DIR ${Visis_ROOT}/src)

add_subdirectory(src)

add_executable(visis visis_main.cc)

target_link_libraries(visis Visis)
