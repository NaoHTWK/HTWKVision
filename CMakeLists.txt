cmake_minimum_required (VERSION 2.6)
project (HTWKVision)

set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_CXX_FLAGS "-O3 -Wall -std=c++11")

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

add_subdirectory(demo)
add_subdirectory(lodepng)
