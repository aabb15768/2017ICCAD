cmake_minimum_required(VERSION 2.8.3)
project(ICCAD)

add_compile_options(-std=c++11)

include_directories(
  include
)

set(net_open_finer_lib_files
        src/net_open_finder.cpp
        src/graph.cpp
	include/net_open_finder.h
        include/graph.h
)

add_library(${PROJECT_NAME} ${net_open_finer_lib_files})
target_link_libraries(${PROJECT_NAME})

add_executable(net_open_finder src/main.cpp)
target_link_libraries(net_open_finder ${PROJECT_NAME})
