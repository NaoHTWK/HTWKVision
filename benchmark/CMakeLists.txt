cmake_minimum_required(VERSION 2.6)

# filesystem also needs system
find_package(Boost REQUIRED program_options filesystem system)

add_executable(memcpy_speed memcpy_speed.cpp memcpy_speed.h)
target_link_libraries(memcpy_speed HTWKVision lodepng ${Boost_LIBRARIES})

add_executable(create_integral_image
    create_integral_image.cpp
    create_integral_image.h
    create_integral_image_naive.cpp
    create_integral_image_naive_unrolled.h
    create_integral_image_naive_unrolled.cpp
    create_integral_image_naive.h
    create_integral_image_sse.cpp
    create_integral_image_sse.h
    create_integral_image_two_rows.cpp
    create_integral_image_two_rows.h
    create_integral_image_planar_yuv.h
    create_integral_image_planar_yuv.cpp
    planar_yuv.h
    planar_yuv.cpp
    )
target_link_libraries(create_integral_image HTWKVision lodepng ${Boost_LIBRARIES})

install(TARGETS memcpy_speed DESTINATION ${CMAKE_SOURCE_DIR}/bin)
install(TARGETS create_integral_image DESTINATION ${CMAKE_SOURCE_DIR}/bin)
install(FILES image.yuv422 DESTINATION ${CMAKE_SOURCE_DIR}/bin)
#install(DIRECTORY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CMAKE_SOURCE_DIR}/bin/data/)
