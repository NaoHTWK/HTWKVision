cmake_minimum_required(VERSION 3.0)
project(tflite-benchmark)

find_package(Boost COMPONENTS program_options filesystem system REQUIRED QUIET)

add_executable(tflite-benchmark tflite-benchmark.cpp)

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address,undefined")
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address,undefined")

target_link_libraries(
    tflite-benchmark
    Vision
    Boost::program_options
    Boost::system
    Boost::filesystem)

set_target_properties(tflite-benchmark PROPERTIES CXX_STANDARD 17)
