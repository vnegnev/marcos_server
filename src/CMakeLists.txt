cmake_minimum_required(VERSION 3.8.2)
project(marcos_server)

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL armv7l)
   # compile in release mode on the ARM
   set (CMAKE_C_FLAGS "-static -O3 -mcpu=cortex-a9 -mtune=cortex-a9 -mfpu=neon -mfloat-abi=hard -Wall -std=c11")
   set (CMAKE_CXX_FLAGS "-static -O3 -mcpu=cortex-a9 -mtune=cortex-a9 -mfpu=neon -mfloat-abi=hard -Wall -std=c++14")
else()
   # compile in debug mode on the host PC for testing
   set (CMAKE_C_FLAGS "-O0 -g -Wall -std=c11")
   set (CMAKE_CXX_FLAGS "-O0 -g -std=c++14")
   # set (CMAKE_C_FLAGS "-static -O3 -Wall -std=c11")
   # set (CMAKE_CXX_FLAGS "-static -O3 -std=c++14")   
endif()

set(SERVER_SRC marcos_server.cpp hardware.cpp iface.cpp mpack/mpack.c)
add_executable(marcos_server ${SERVER_SRC})
