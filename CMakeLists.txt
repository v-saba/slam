cmake_minimum_required(VERSION 3.30)
project(slam)

set(CMAKE_CXX_STANDARD 20)
set(OpenCV_DIR "/opt/homebrew/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

# Require nlohmann_json package
find_package(nlohmann_json REQUIRED)

# Main SLAM executable
add_executable(slam main.cpp
        slam_step.cpp
        slam_step.h)

target_link_libraries(slam ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

# Camera calibration executable
add_executable(calibrate calibrate.cpp)

target_link_libraries(calibrate ${OpenCV_LIBS} nlohmann_json::nlohmann_json)
