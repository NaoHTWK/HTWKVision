cmake_minimum_required (VERSION 2.6)

include_directories("..")

add_executable(demo demo.cpp)
target_link_libraries(demo HTWKVision lodepng)
