cmake_minimum_required (VERSION 2.6)

# filesystem also needs system
find_package(Boost COMPONENTS program_options filesystem system REQUIRED)

include_directories("..")

add_executable(demo demo.cpp)
target_link_libraries(demo HTWKVision lodepng m stdc++ ${Boost_LIBRARIES})
