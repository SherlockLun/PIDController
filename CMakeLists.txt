cmake_minimum_required(VERSION 3.10)
project(PIDController)

set(CMAKE_CXX_STANDARD 20)

include_directories(inc)

add_executable(PIDController
    src/main.cpp
    src/controller.cpp)
