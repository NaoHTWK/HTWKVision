cmake_minimum_required (VERSION 2.6)

# filesystem also needs system
find_package(Boost 1.55 REQUIRED program_options filesystem system atomic)

message("${Boost_LIBRARIES}")

add_executable(demo demo.cpp)

target_link_libraries(
    demo
    HTWKVision
    htwkimagetools
    HTWKNeuronalNet
    ${Boost_LIBRARIES})

if(NOT DEFINED NAO_HW_VERSION)
    install(TARGETS demo DESTINATION ${CMAKE_SOURCE_DIR}/bin)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../data/ DESTINATION ${CMAKE_SOURCE_DIR}/bin/data/)
else()
    install(TARGETS demo DESTINATION "${DEPLOY_DIR}/home")
endif()
