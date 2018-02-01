cmake_minimum_required(VERSION 2.8)
project(HTWKVisionTests)

add_subdirectory(messages)

find_package(CppUnit)
find_package(Boost REQUIRED filesystem system)

#include_directories(.., ../lodepng)

add_executable(HTWKVisionTests
    htwkvisiontests.cpp
    testimagedata.cpp
    testimageloader.cpp
    testutils.cpp
    main.cpp
)

target_link_libraries(HTWKVisionTests
    HTWKVision
    lodepng
    visionresult
    ${CppUnit_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(testfilecreator
    testimagedata.cpp
    testimageloader.cpp
    testutils.cpp
    testfilecreator.cpp
)

target_link_libraries(testfilecreator
    visionresult
    HTWKVision
    lodepng
    ${Boost_LIBRARIES}
)

message(${CMAKE_SOURCE_DIR}/data/)
message(${CMAKE_BINARY_DIR}/data/)

install(TARGETS HTWKVisionTests DESTINATION ${CMAKE_SOURCE_DIR}/bin)
install(TARGETS testfilecreator DESTINATION ${CMAKE_SOURCE_DIR}/bin)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CMAKE_SOURCE_DIR}/bin/data/)
