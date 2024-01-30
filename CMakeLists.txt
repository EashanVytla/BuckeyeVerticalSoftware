cmake_minimum_required(VERSION 3.15)

# Specify your project's name
project(BuckeyeVerticalSoftware)

# Specify at least C++11, better C++18
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable strict handling of warnings
add_definitions("-Wall -Wextra")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(test
    src/livefeed/test.cpp
    src/livefeed/Bytes.cpp
)
