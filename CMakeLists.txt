cmake_minimum_required(VERSION 3.0.2)
project(my_A_star)


add_executable(a_star main.cpp A_star.cpp)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
