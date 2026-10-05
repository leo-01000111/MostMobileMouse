# JNI bridge for the C++ core in PC-part/core (shared with the parity replay tool).
cmake_minimum_required(VERSION 3.22)
project(deskmouse_jni CXX)

# The engine runs per frame in real time: optimize even in debug builds (AGP builds debug with -O0).
add_compile_options(-O2)

find_package(OpenCV REQUIRED CONFIG)  # prefab from the org.opencv:opencv AAR
set(DESKMOUSE_OPENCV_TARGET OpenCV::opencv_java4)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../../../PC-part/core ${CMAKE_BINARY_DIR}/core)

add_library(deskmouse_jni SHARED jni.cpp)
target_link_libraries(deskmouse_jni PRIVATE deskmouse_core log)
