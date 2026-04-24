cmake_minimum_required(VERSION 3.10)
project(hw11)

set(CMAKE_C_STANDARD 99)

add_subdirectory(libtcache)

add_executable(hw11 main.c)
target_link_libraries(hw11 tcache)
