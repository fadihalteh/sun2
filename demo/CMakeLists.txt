cmake_minimum_required(VERSION 3.16)
project(pipeline_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------
# Usage from the repo root:
#   cmake -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build --target pipeline_demo
# -----------------------------------------------------------------------

add_executable(pipeline_demo main.cpp)

target_link_libraries(pipeline_demo
    PRIVATE
        solar_vision      # SunTracker
        solar_control     # Controller, Kinematics3RRS
        solar_sensors     # SimulatedPublisher
        solar_common      # Logger, Types
)

target_include_directories(pipeline_demo
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
)
