cmake_minimum_required(VERSION 3.16)

project(SolarStewartTracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SOLAR_ENABLE_TESTS      "Build tests"                                    ON)
option(SOLAR_ENABLE_HW_TESTS   "Register real hardware smoke tests"            OFF)
option(SOLAR_ENABLE_QT         "Build Qt GUI target if Qt5 is available"        ON)
option(SOLAR_TRY_LIBCAMERA     "Try to enable libcamera backend"                ON)
option(SOLAR_TRY_OPENCV        "Try to enable OpenCV support"                   ON)
option(SOLAR_ENABLE_COVERAGE   "Enable coverage flags where supported"          OFF)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

function(solar_enable_coverage target_name)
    if(SOLAR_ENABLE_COVERAGE AND NOT MSVC)
        target_compile_options(${target_name} PRIVATE -O0 -g --coverage)
        target_link_options(${target_name} PRIVATE --coverage)
    endif()
endfunction()

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

set(SOLAR_HAVE_OPENCV 0)
if(SOLAR_TRY_OPENCV)
    find_package(OpenCV QUIET)
    if(OpenCV_FOUND)
        set(SOLAR_HAVE_OPENCV 1)
        message(STATUS "OpenCV found: optional OpenCV support enabled")
    else()
        message(STATUS "OpenCV not found: optional OpenCV support disabled")
    endif()
else()
    message(STATUS "OpenCV probing disabled by SOLAR_TRY_OPENCV=OFF")
endif()

set(SOLAR_HAVE_LIBCAMERA 0)
if(SOLAR_TRY_LIBCAMERA)
    pkg_check_modules(LIBCAMERA QUIET IMPORTED_TARGET libcamera)
    if(LIBCAMERA_FOUND AND SOLAR_HAVE_OPENCV)
        set(SOLAR_HAVE_LIBCAMERA 1)
        message(STATUS "libcamera found with OpenCV: optional libcamera backend enabled")
    elseif(LIBCAMERA_FOUND AND NOT SOLAR_HAVE_OPENCV)
        message(STATUS "libcamera found but OpenCV missing: libcamera backend disabled")
    else()
        message(STATUS "libcamera not found: optional libcamera backend disabled")
    endif()
else()
    message(STATUS "libcamera probing disabled by SOLAR_TRY_LIBCAMERA=OFF")
endif()

set(SOLAR_HAVE_GPIOD_DEMO 0)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/libgpiod_event_demo/CMakeLists.txt")
    add_subdirectory(external/libgpiod_event_demo)
    set(SOLAR_HAVE_GPIOD_DEMO 1)
    message(STATUS "external/libgpiod_event_demo found")
else()
    message(STATUS "external/libgpiod_event_demo not found: GPIO demo support disabled")
endif()

if(SOLAR_HAVE_LIBCAMERA)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/libcamera2opencv/libcam2opencv.cpp")
        add_library(cam2opencv STATIC
            external/libcamera2opencv/libcam2opencv.cpp
        )
        target_include_directories(cam2opencv
            PUBLIC
                ${CMAKE_CURRENT_SOURCE_DIR}/external/libcamera2opencv
                ${OpenCV_INCLUDE_DIRS}
                ${LIBCAMERA_INCLUDE_DIRS}
        )
        target_link_libraries(cam2opencv
            PUBLIC
                PkgConfig::LIBCAMERA
                ${OpenCV_LIBS}
        )
        target_compile_options(cam2opencv PRIVATE ${LIBCAMERA_CFLAGS_OTHER})
        message(STATUS "external/libcamera2opencv found")
    else()
        message(STATUS "external/libcamera2opencv not found: libcamera backend disabled")
        set(SOLAR_HAVE_LIBCAMERA 0)
    endif()
endif()

set(SOLAR_HAVE_LIBCAMERA ${SOLAR_HAVE_LIBCAMERA} CACHE INTERNAL "")
set(SOLAR_HAVE_OPENCV    ${SOLAR_HAVE_OPENCV}    CACHE INTERNAL "")
set(SOLAR_HAVE_GPIOD_DEMO ${SOLAR_HAVE_GPIOD_DEMO} CACHE INTERNAL "")

if(SOLAR_ENABLE_TESTS)
    enable_testing()
endif()

add_subdirectory(src/common)
add_subdirectory(src/hal)
add_subdirectory(src/control)
add_subdirectory(src/actuators)
add_subdirectory(src/sensors)
add_subdirectory(src/vision)
add_subdirectory(src/system)
add_subdirectory(src/app)

if(SOLAR_ENABLE_QT)
    add_subdirectory(src/qt)
endif()

add_executable(solar_tracker src/main.cpp)
target_include_directories(solar_tracker PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(solar_tracker PRIVATE solar_app)
solar_enable_coverage(solar_tracker)
