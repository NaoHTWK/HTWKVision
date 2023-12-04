cmake_minimum_required(VERSION 3.16)
project(Vision)

option(BUILD_BENCHMARK "Build the benchmark." ON)
option(VISION_ACTIVATE_VISUALIZATION "Build with visualization." OFF)
option(BUILD_DEMO_APP "Build the demo application" ON)

find_package(Boost COMPONENTS filesystem REQUIRED QUIET)

set(LIBS
    stdc++
    rt
    m
    pthread
)

add_library(${PROJECT_NAME} SHARED
    ball_classifier_upper_cam.cpp
    ball_classifier_upper_cam.h
    ball_pre_classifier_upper_cam.cpp
    ball_pre_classifier_upper_cam.h
    ball_detector.h
    ball_feature_extractor.cpp
    ball_feature_extractor.h
    base_detector.h
    bounding_box.h
    ellifit.cpp
    ellifit.h
    ellipse.h
    field_border_detector.cpp
    field_border_detector.h
    field_color_detector.cpp
    field_color_detector.h
    image_preprocessor.cpp
    image_preprocessor.h
    htwk_vision.cpp
    htwk_vision.h
    htwk_vision_config.cpp
    htwk_vision_config.h
    hypotheses_generator.h
    hypotheses_generator_blur.cpp
    hypotheses_generator_blur.h
    integral_image.cpp
    integral_image.h
    imagedebughelper.cpp
    imagedebughelper.h
    jersey_detection.cpp
    jersey_detection.h
    lc_centercirclepoints_detector.h
    lc_centercirclepoints_detector.cpp
    lc_obstacle_detection.h
    lc_obstacle_detection.cpp
    lc_scrambled_camera_detector.h
    lc_scrambled_camera_detector.cpp
    linecross.h
    line_detector.cpp
    line_detector.h
    object_detector_lowercam.cpp
    object_detector_lowercam.h
    object_detector_lowercam_hyp_gen.cpp
    object_detector_lowercam_hyp_gen.h
    object_hypothesis.h
    penaltyspot_detector.h
    range_check.h
    ransac_ellipse_fitter.cpp
    ransac_ellipse_fitter.h
    region_classifier.cpp
    region_classifier.h
    uc_ball_hyp_gen.h
    uc_ball_hyp_gen.cpp
    uc_centercirclepoints_detector.h
    uc_centercirclepoints_detector.cpp
    uc_dirty_camera_detector.h
    uc_dirty_camera_detector.cpp
    uc_goalpost_detector.h
    uc_goalpost_detector.cpp
    uc_penaltyspot_classifier.h
    uc_penaltyspot_classifier.cpp
    uc_robot_detector.h
    uc_robot_detector.cpp
    lineedge.cpp
    lineedge.h
    htwkyuv422image.cpp
    htwkyuv422image.h
    htwkcolorconversion.cpp
    htwkcolorconversion.h
    localization_utils.cpp
    localization_utils.h
    htwkpngimagesaver.cpp
    htwkpngimagesaver.h
    cam_offset.cpp
    cam_offset.h
    htwkpngimageprovider.cpp
    htwkpngimageprovider.h
    cam_pose.cpp
    cam_pose.h
)

if(VISION_ACTIVATE_VISUALIZATION)
    target_compile_definitions(${PROJECT_NAME} PUBLIC VISION_ACTIVATE_VISUALIZATION)
endif()


add_subdirectory(tfliteexecuter)

target_link_libraries(${PROJECT_NAME} ${LIBS}
    tfliteexecuter
    ${CMAKE_DL_LIBS}
    Boost::filesystem
)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)

# if(BUILD_BENCHMARK)
#     add_subdirectory(tflite-benchmark)
# endif()

if(BUILD_DEMO_APP)
    add_subdirectory(demo)
endif()

