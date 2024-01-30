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

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/livefeed/hex)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/livefeed/crc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/livefeed/commandType)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/livefeed/message)


add_executable(test

    src/livefeed/commandType/commandType.cpp
    # src/livefeed/${cmd}/${cmd}.h

    src/livefeed/crc/crc.cpp
    # src/livefeed/crc/crc.h

    src/livefeed/hex/hex.cpp
    # src/livefeed/hex/hex.h

    src/livefeed/message/message.cpp
    # src/livefeed/message/message.h

    src/livefeed/test.cpp

)
