find_package(Boost COMPONENTS program_options filesystem system REQUIRED QUIET)

add_executable(demo demo.cpp)
add_executable(scaled_image_writer scaled_image_writer.cpp)

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address,undefined")
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address,undefined")

target_link_libraries(
    demo
    Vision
    Boost::program_options
    Boost::system
    Boost::filesystem)

target_link_libraries(
    scaled_image_writer
    Vision
    Boost::program_options
    Boost::system
    Boost::filesystem)

set_target_properties(demo PROPERTIES CXX_STANDARD 17)
set_target_properties(scaled_image_writer PROPERTIES CXX_STANDARD 17)
