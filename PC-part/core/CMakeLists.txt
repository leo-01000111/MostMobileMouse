# Desk mouse C++ core (port of PC-part/proto/deskmouse engine + front end).
# Built from the Android app (Phone-part/recorder/src/main/cpp) with OpenCV from the Maven AAR (prefab).
# The including project must provide the OpenCV target in DESKMOUSE_OPENCV_TARGET.
cmake_minimum_required(VERSION 3.22)
project(deskmouse_core CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(deskmouse_core STATIC src/config.cpp src/frontend.cpp src/engine.cpp)
target_include_directories(deskmouse_core PUBLIC include PRIVATE src)
target_link_libraries(deskmouse_core PUBLIC ${DESKMOUSE_OPENCV_TARGET})

option(DESKMOUSE_BUILD_REPLAY "build the parity replay tool" ON)
if(DESKMOUSE_BUILD_REPLAY)
    add_executable(deskmouse_replay tools/replay.cpp)
    target_link_libraries(deskmouse_replay PRIVATE deskmouse_core)
endif()
