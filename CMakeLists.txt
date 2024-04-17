cmake_minimum_required(VERSION 3.15)

# Specify your project's name
project(BuckeyeVerticalSoftware LANGUAGES CXX)

# Specify at least C++11, better C++18
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable strict handling of warnings
add_definitions("-Wall -Wextra")

# Finds MAVSDK when installed system wide.
find_package(MAVSDK REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

list(APPEND INCLUDE_DIRS
     ${OpenCV_INCLUDE_DIRS}
     include
)

include_directories(${INCLUDE_DIRS})

add_executable(main
               src/main_test.cpp
               src/agent.cpp
               src/detect.cpp
               src/camera_server.cpp
)

# Specify your app's executable name and a list of linked libraries
target_link_libraries(main
                      MAVSDK::mavsdk
                      ${OpenCV_LIBS}
)