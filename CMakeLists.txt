cmake_minimum_required(VERSION 3.3)
project(qemu_dev)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

file(GLOB_RECURSE src "*.c")

set(SOURCES
    ${src})

include_directories(include)

add_custom_target(qemu_dev COMMAND make
        CLION_EXE_DIR=${PROJECT_BINARY_DIR})

