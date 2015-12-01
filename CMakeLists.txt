cmake_minimum_required (VERSION 2.6)
project (HTWKVision)

option(BUILD_DEMO_APP "Build the demo application which processes a PNG and shows what it detected." OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -std=c++11 -ffast-math -march=native -mfpmath=sse -funroll-loops -msse3 -fomit-frame-pointer")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -std=c++11 -ffast-math -mfpmath=sse -funroll-loops -msse3 -fomit-frame-pointer -Rpass-analysis=loop-vectorize")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -std=c++11 -ffast-math -mfpmath=sse -funroll-loops -msse3 -fno-omit-frame-pointer")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -std=c++11 -ffast-math -mfpmath=sse -funroll-loops -msse3 -fsanitize=undefined,address")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --verbose -Wl,--whole-archive -L/usr/lib/llvm-3.6/lib/clang/3.6.0/lib/linux -lclang_rt.ubsan_cxx-x86_64 -Wl,--no-whole-archive")

include_directories(".")
add_library(HTWKVision STATIC
  ball_detector.cpp
  classifier.cpp
  ellifit.cpp
  feet_detector.cpp
  field_color_detector.cpp
  field_detector.cpp
  goal_detector.cpp
  htwk_vision.cpp
  jersey_color_detector.cpp
  line_detector.cpp
  lineedge.cpp
  near_obstacle_detector.cpp
  ransac_ellipse_fitter.cpp
  region_classifier.cpp
  robot_area_detector.cpp
  robot_classifier.cpp)

if(BUILD_DEMO_APP)
    add_subdirectory(demo)
    add_subdirectory(lodepng)
endif(BUILD_DEMO_APP)
