cmake_minimum_required(VERSION 3.15)

# Specify your project's name
project(BuckeyeVerticalSoftware)

# Specify at least C++11, better C++18
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable strict handling of warnings
add_definitions("-Wall -Wextra")

# Finds MAVSDK when installed system wide.
find_package(MAVSDK REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Find Tesseract
find_package(PkgConfig REQUIRED)

# Specify your app's executable name, and list of source files used to create it.
add_executable(MavLinkTest
    src/MavLinkTest.cpp
    src/PWMUtils.cpp
    # ... any other source files
)

add_executable(MavLinkTestSim
    src/MavLinkTestSim.cpp
)

add_executable(PWMPeriodFinder
    src/PWMPeriodFinder.cpp
)

add_executable(OCRTest
    src/OCRTest.cpp
)

# Link libraries
target_link_libraries(OCRTest ${OpenCV_LIBS} /usr/local/lib/libtesseract.so)

# Specify your app's executable name and a list of linked libraries
target_link_libraries(MavLinkTest
    MAVSDK::mavsdk             #All apps link against mavsdk library
)

# Specify your app's executable name and a list of linked libraries
target_link_libraries(MavLinkTestSim
    MAVSDK::mavsdk             #All apps link against mavsdk library
)