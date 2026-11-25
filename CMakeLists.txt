cmake_minimum_required(VERSION 3.16)

project(AutonomousDroneEngineering
    VERSION 0.1.0
    DESCRIPTION "Hardware reference stack for an autonomous quadrotor"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(ADE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

find_package(Threads REQUIRED)

# Collect every subsystem driver under hardware/. The drivers are written
# against the Linux user-space device APIs (spidev, i2c-dev, termios) and have
# no external dependencies, so the library builds on any Linux host: suitable
# for bench bring-up rigs, HIL setups, and embedded Linux flight computers.
# Include paths mirror the tree: #include "sensors/imu/imu_interface.hpp".
file(GLOB_RECURSE ADE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/*.cpp")
file(GLOB_RECURSE ADE_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/*.hpp")

add_library(drone_hardware STATIC ${ADE_SOURCES} ${ADE_HEADERS})
target_include_directories(drone_hardware
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/hardware")
target_link_libraries(drone_hardware PUBLIC Threads::Threads)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(drone_hardware PRIVATE
        -Wall -Wextra -Wpedantic -Wshadow)
    if(ADE_WARNINGS_AS_ERRORS)
        target_compile_options(drone_hardware PRIVATE -Werror)
    endif()
endif()

# Install the static library and the driver headers (tree structure kept).
install(TARGETS drone_hardware
    ARCHIVE DESTINATION lib)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/hardware/"
    DESTINATION include/ade
    FILES_MATCHING PATTERN "*.hpp")
