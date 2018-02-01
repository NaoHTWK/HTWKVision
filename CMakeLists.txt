cmake_minimum_required(VERSION 2.6)
project(HTWKVision)


set(CMAKE_CXX_STANDARD 14)


# To find FindCppUnit
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Caffe REQUIRED)
find_package(Boost COMPONENTS program_options signals system atomic REQUIRED)

option(BUILD_DEMO_APP "Build the demo application which processes a PNG and shows what it detected." ON)
option(BUILD_TESTS "Build the tests." OFF)
option(BUILD_BENCHMARK "Build the benchmark." OFF)

# Use ccache when available
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)

    # When CCache is used an unused argument is added and color diagnostic is deactivated
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qunused-arguments -fcolor-diagnostics")
    endif()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -std=c++14 -ffast-math -march=atom -mfpmath=sse -funroll-loops -msse3 -mssse3 -fomit-frame-pointer -Wno-unused-local-typedef")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -std=c++14 -ffast-math -march=native -mfpmath=sse -funroll-loops -msse3 -mssse3 -fomit-frame-pointer -Wno-unused-local-typedef")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -std=c++11 -ffast-math -mfpmath=sse -funroll-loops -msse3 -fomit-frame-pointer -Rpass-analysis=loop-vectorize")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -O0 -Wall -std=c++11 -ffast-math -mfpmath=sse -funroll-loops -msse3 -mssse3 -fno-omit-frame-pointer -fno-inline")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall -std=c++11 -ffast-math -mfpmath=sse -funroll-loops -msse3 -fsanitize=undefined,address")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --verbose -Wl,--whole-archive -L/usr/lib/llvm-3.6/lib/clang/3.6.0/lib/linux -lclang_rt.ubsan_cxx-x86_64 -Wl,--no-whole-archive")

set(LIBS
    stdc++
    rt
    m
    pthread
)


if(NOT Caffe_FOUND)
    message(FATAL_ERROR "CAFFE wasn't found!!!!")
endif()

if(Caffe_FOUND)
    message("Caffe was found in ${Caffe_INCLUDE_DIRS}")

    include_directories(${Caffe_INCLUDE_DIRS})
    set(LIBS ${LIBS} ${Caffe_LIBRARIES}) # ${OpenCV_LIBS})
    add_definitions(${Caffe_DEFINITIONS})
    message("LIBS ${LIBS}")
endif()

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  neuralnet/classifier.cpp
  neuralnet/classifier.h
  neuralnet/classifier_relu.cpp
  neuralnet/classifier_relu.h
  neuralnet/convolutions.cpp
  neuralnet/convolutions.h
  neuralnet/layer.h
  neuralnet/layer_activations.h
  neuralnet/pooling_result.h

  ball_detector.h
  ball_detector.cpp
  ball_detector_legacy.h
  ball_detector_legacy.cpp
  ball_feature_extractor.cpp
  ball_feature_extractor.h
  base_detector.h
  color.h
  ellifit.cpp
  ellifit.h
  ellipse.h
  ext_math.h
  feet_detector.cpp
  feet_detector.h
  field_color_detector.cpp
  field_color_detector.h
  field_detector.cpp
  field_detector.h
  goal_detector.cpp
  goal_detector.h
  goalpost.h
  htwk_vision.cpp
  htwk_vision.h
  htwk_vision_config.h
  hypotheses_generator.h
  hypotheses_generator.cpp
  hypotheses_generator_blocks.cpp
  hypotheses_generator_blocks.h
  hypotheses_generator_blur.cpp
  hypotheses_generator_blur.h
  hypotheses_generator_scanlines.cpp
  hypotheses_generator_scanlines.h
  integral_image.cpp
  integral_image.h
  jersey_color_detector.cpp
  jersey_color_detector.h
  line_detector.cpp
  line_detector.h
  line.h
  lineedge.cpp
  lineedge.h
  linegroup.h
  linesegment.h
  near_obstacle_detector.cpp
  near_obstacle_detector.h
  object_hypothesis.h
  object_detector.h
  object_detector.cpp
  projectionutil.h
  ransac_ellipse_fitter.cpp
  ransac_ellipse_fitter.h
  rect.h
  region_classifier.cpp
  region_classifier.h
  robot_area_detector.cpp
  robot_area_detector.h
  robot_detector.cpp
  robot_detector.h
  robot_classifier.cpp
  robot_classifier.h
  team_membership.h
)

target_link_libraries(${PROJECT_NAME} ${LIBS})
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/neuralnet
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/eigen
        ${Caffe_INCLUDE_DIRS}
        ..
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS ${PROJECT_NAME} DESTINATION "lib")

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif()

if(BUILD_DEMO_APP)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libhtwkimagetools libhtwkimagetools)
    add_subdirectory(demo)
endif()

