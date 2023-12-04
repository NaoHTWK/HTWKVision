cmake_minimum_required(VERSION 3.16)
project(tfliteexecuter)

find_package(Boost COMPONENTS filesystem REQUIRED QUIET)

set(LIBS
    stdc++
    rt
    m
    pthread
)

add_library(${PROJECT_NAME} SHARED
    tfliteexecuter.cpp
    tfliteexecuter.h
)

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    message("tflite: I found the 32bit version")
    message(FATAL_ERROR "Not supportet anymore!")
elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
    message("tflite: I found the 64bit version")
    add_library(tensorflow-lite STATIC IMPORTED)
    add_library(ruy STATIC IMPORTED)
    add_library(farmhash STATIC IMPORTED)
    add_library(flatbuffers STATIC IMPORTED)
    add_library(fft2d_fftsg STATIC IMPORTED)
    add_library(fft2d_fftsg2d STATIC IMPORTED)
    add_library(xnnpack STATIC IMPORTED)
    add_library(cpuinfo STATIC IMPORTED)
    add_library(pthreadpool STATIC IMPORTED)

    set_target_properties(tensorflow-lite PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/../tflite/x64/libtensorflow-lite.a)
    set_target_properties(ruy PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/../tflite/x64/libruy.a)
    set_target_properties(farmhash PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/../tflite/x64/libfarmhash.a)
    set_target_properties(flatbuffers PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/../tflite/x64/libflatbuffers.a)
    set_target_properties(fft2d_fftsg PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/../tflite/x64/libfft2d_fftsg.a)
    set_target_properties(fft2d_fftsg2d PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/../tflite/x64/libfft2d_fftsg2d.a)
    set_target_properties(xnnpack PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/../tflite/x64/libXNNPACK.a)
    set_target_properties(cpuinfo PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/../tflite/x64/libcpuinfo.a)
    set_target_properties(pthreadpool PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/../tflite/x64/libpthreadpool.a)
else()
    message(FATAL_ERROR "We couldn't identify the needed tflite version.")
endif()
set_target_properties(tensorflow-lite PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/../tflite")

target_link_libraries(${PROJECT_NAME} ${LIBS}
    tensorflow-lite
    ruy farmhash flatbuffers fft2d_fftsg fft2d_fftsg2d
    xnnpack cpuinfo pthreadpool ${CMAKE_DL_LIBS}
    Boost::filesystem
)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
