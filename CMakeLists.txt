cmake_minimum_required(VERSION 3.15)

# Specify your project's name
project(BuckeyeVerticalSoftware)

# Specify at least C++11, better C++18
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(cmd "command")

# Enable strict handling of warnings
add_definitions("-Wall -Wextra")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(include_directories(/Library/Developer/CommandLineTools/usr/include))

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/livefeed/sdk/headers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/livefeed/sdk/sources)


file(GLOB SRC_FILES "src/livefeed/sdk/sources/*.cpp")

add_executable(test
    src/livefeed/test.cpp
    ${SRC_FILES} 
)


add_executable(test_udpclient
    src/livefeed/test_udpclient.cpp
    ${SRC_FILES} 
)

add_executable(test_udpserver
    src/livefeed/test_udpserver.cpp
    ${SRC_FILES} 
)


