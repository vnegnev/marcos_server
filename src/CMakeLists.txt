cmake_minimum_required(VERSION 3.8.2)
project(marcos_server)

set (CMAKE_CXX_FLAGS "-static -O3 -march=armv7-a -mcpu=cortex-a9 -mtune=cortex-a9 -mfpu=neon -mfloat-abi=hard -Wall -std=c++14")

set(SERVER_SRC marcos_server.cpp)
add_executable(marcos_server ${SERVER_SRC})
