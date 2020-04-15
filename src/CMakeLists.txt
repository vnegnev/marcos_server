cmake_minimum_required(VERSION 3.8.2)
project(marcos_server)

# Set this false or comment it out to compile locally on x86 for testing
# set(arm_compile true)

if (arm_compile)
   set (CMAKE_C_FLAGS "-static -O3 -march=armv7-a -mcpu=cortex-a9 -mtune=cortex-a9 -mfpu=neon -mfloat-abi=hard -Wall -std=c11")
   set (CMAKE_CXX_FLAGS "-static -O3 -march=armv7-a -mcpu=cortex-a9 -mtune=cortex-a9 -mfpu=neon -mfloat-abi=hard -Wall -std=c++14")
else()
   set (CMAKE_C_FLAGS "-O0 -g -Wall -std=c11")
   set (CMAKE_CXX_FLAGS "-O0 -g -std=c++14")
endif()

set(SERVER_SRC marcos_server.cpp hardware.cpp iface.cpp mpack/mpack.c)
add_executable(marcos_server ${SERVER_SRC})
