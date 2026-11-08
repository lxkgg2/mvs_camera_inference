cmake_minimum_required(VERSION 3.16)
project(mvs_yolo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 告诉 CMake 去哪找 OpenCV（你机器上的实际路径）
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui dnn)

set(MVS_ROOT "/opt/MVS" CACHE PATH "MVS SDK root")
find_path(MVS_INCLUDE_DIR NAMES MvCameraControl.h HINTS "${MVS_ROOT}/include")
find_library(MVS_LIBRARY NAMES MvCameraControl
        HINTS "${MVS_ROOT}/lib/64" "${MVS_ROOT}/lib/aarch64")

if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
    message(FATAL_ERROR "找不到 MVS SDK，请用 -DMVS_ROOT=你的路径 指定")
endif()

add_executable(mvs_yolo mvs_yolo.cpp)
target_include_directories(mvs_yolo PRIVATE ${MVS_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(mvs_yolo PRIVATE ${MVS_LIBRARY} ${OpenCV_LIBS})

get_filename_component(MVS_RUNTIME_DIR "${MVS_LIBRARY}" DIRECTORY)
set_target_properties(mvs_yolo PROPERTIES BUILD_RPATH "${MVS_RUNTIME_DIR}")
