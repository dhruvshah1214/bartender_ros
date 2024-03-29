# Project's name
PROJECT ( hardware_test C CXX )

# Specify the minimum version for CMake

cmake_minimum_required(VERSION 3.1)

add_definitions(-std=c++11)

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package (Threads REQUIRED)

# The following folder will be included
include_directories("${PROJECT_SOURCE_DIR}/include")

add_executable(example ${PROJECT_SOURCE_DIR}/example.cpp)
add_executable(diagserver ${PROJECT_SOURCE_DIR}/diagserver.cpp)
add_executable(tunepid ${PROJECT_SOURCE_DIR}/tunepid.cpp)

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
	target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCore.a)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_Phoenix.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCCI.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCanutils.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCore.a)
    target_link_libraries(diagserver ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_Phoenix.a)
    target_link_libraries(diagserver ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCCI.a)
    target_link_libraries(diagserver ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(diagserver ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(diagserver ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCanutils.a)
    target_link_libraries(diagserver ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCore.a)
    target_link_libraries(tunepid ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_Phoenix.a)
    target_link_libraries(tunepid ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCCI.a)
    target_link_libraries(tunepid ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(tunepid ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(tunepid ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCanutils.a)
    target_link_libraries(tunepid ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCore.a)
else()
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)
    target_link_libraries(example ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCore.a)
endif()

target_link_libraries(example Threads::Threads)
target_link_libraries(diagserver Threads::Threads)
target_link_libraries(tunepid Threads::Threads)


add_subdirectory(socketcan-cpp)
target_link_libraries(example socketcan_cpp)

