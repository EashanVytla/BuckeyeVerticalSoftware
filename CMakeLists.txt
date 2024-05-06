cmake_minimum_required(VERSION 3.15)

set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

# Specify your project's name
project(BuckeyeVerticalSoftware LANGUAGES CXX CUDA)

# Specify at least C++11, better C++18
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable strict handling of warnings
add_definitions("-Wall -Wextra")

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# Finds MAVSDK when installed system wide.
find_package(MAVSDK REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# CUDA
find_package(CUDA REQUIRED)

message(STATUS "CUDA Libs: \n${CUDA_LIBRARIES}\n")
get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")

# TensorRT
set(TensorRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu)
set(TensorRT_LIBRARIES /usr/lib/aarch64-linux-gnu)

list(APPEND INCLUDE_DIRS
     ${CUDA_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS}
     ${TensorRT_INCLUDE_DIRS}
     include
)

include_directories(${INCLUDE_DIRS})

list(APPEND ALL_LIBS
     ${CUDA_LIBRARIES}
     ${CUDA_LIB_DIR}
     ${OpenCV_LIBRARIES}
     ${TensorRT_LIBRARIES}
)

add_executable(main
               src/main_test.cpp
               src/agent.cpp
               src/detect.cpp
)

add_executable(servoTest
	src/ServoTester.cpp
)

add_executable(detectTest
               src/detectTest.cpp
               src/detect.cpp
)

# Specify your app's executable name and a list of linked libraries
target_link_libraries(main
                      MAVSDK::mavsdk
                      nvinfer
                      nvinfer_plugin
                      ${CUDA_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${ALL_LIBS}
)


target_link_libraries(servoTest
                      MAVSDK::mavsdk
)

# Specify your app's executable name and a list of linked libraries
target_link_libraries(detectTest
                      MAVSDK::mavsdk
                      nvinfer
                      nvinfer_plugin
                      ${CUDA_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${ALL_LIBS}
)
