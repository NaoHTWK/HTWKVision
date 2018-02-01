cmake_minimum_required(VERSION 2.8)

#
# Add Protobuf as dependency
#
include(FindProtobuf)

#We already have set all directories we need.
#find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

#
# Find the protoc Executable. It's copied from FindProtobuf.cmake
# which does not find it.
#

#find_program(PROTOBUF_PROTOC_EXECUTABLE
#    NAMES protoc PATHS ${PROTOBUF_ROOT}/bin
#    DOC "The Google Protocol Buffers Compiler"
#)
mark_as_advanced(PROTOBUF_PROTOC_EXECUTABLE)

#
# Generate source files
#
file(GLOB ProtoFiles "${PROTO_DIR}/visionresult.proto")
protobuf_generate_cpp(ProtoSources ProtoHeaders ${ProtoFiles})

#
# Build a static library which can be linked against our executables.
#
add_library(visionresult STATIC ${ProtoSources} ${ProtoHeaders})
target_link_libraries(visionresult protobuf)

#
# We want to be able to include the messages later
#
set(VisionProtobufIncludePath ${CMAKE_CURRENT_BINARY_DIR}
        CACHE INTERNAL "Path to generated protobuf files.")

target_include_directories(visionresult PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

