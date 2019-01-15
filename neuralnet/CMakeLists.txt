project(HTWKNeuronalNet)

#
# Generate source files
#
set(src
    classifier.cpp
    classifier.h
    classifier_relu.cpp
    classifier_relu.h
    convolutions.cpp
    convolutions.h
)

add_library(${PROJECT_NAME} SHARED ${src})
target_link_libraries(${PROJECT_NAME} Eigen)
target_include_directories(${PROJECT_NAME} PUBLIC . ../include)

install(TARGETS ${PROJECT_NAME} DESTINATION "${DEPLOY_DIR}/home/lib")
