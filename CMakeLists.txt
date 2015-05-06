cmake_minimum_required (VERSION 2.6)
project (HTWKVision)

option(BUILD_DEMO_APP "Build the demo application which process a PNG and shows it had detected." ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -std=c++11")

include_directories(".")
add_library(HTWKVision SHARED
  ball_detector.cpp
  classifier.cpp
  ellifit.cpp
  feet_detector.cpp
  field_color_detector.cpp
  field_detector.cpp
  goal_detector.cpp
  htwk_vision.cpp
  line_detector.cpp
  lineedge.cpp
  ransac_ellipse_fitter.cpp
  region_classifier.cpp
  robot_area_detector.cpp
  robot_classifier.cpp)

if(BUILD_DEMO_APP)
    add_subdirectory(demo)
    add_subdirectory(lodepng)
endif(BUILD_DEMO_APP)
