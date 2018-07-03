cmake_minimum_required(VERSION 2.8)
project(slam-MapGen)

set(CMAKE_CXX_STANDARD 11)

set(Boost_USE_STATIC_LIBS ON)
set(CMAKE_CXX_FLAGS "-pthread")

# config logging (auto-generation for different release build)
MESSAGE("Generating logging_util.h for logging")
configure_file("src/logging/logging_util.h.in" "${PROJECT_SOURCE_DIR}/src/logging/logging_util.h")

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
)
# OpenCV
find_package( OpenCV REQUIRED)
# DBoW2
find_package( DBoW2 REQUIRED)
include_directories(${DBoW2_INCLUDE_DIRS})
find_package( DLib REQUIRED)
include_directories(${DLib_INCLUDE_DIRS})
# Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
ADD_DEFINITIONS(-DPANGOLIN)

# src
include_directories(src/common
        src/loop-closure
        src/logging
        src/config
        src/tools
        src/visualization
        src/DBoW2-ext
        third-party/ORB2_SLAM
        )

# common
file(GLOB common_src src/common/*(.h|.cpp))
file(GLOB dow2_ext_src src/DBoW2-ext/*(.h|.cpp))
file(GLOB logging_src src/logging/*.cpp)
file(GLOB config_src src/config/*(.h|.cpp))
file(GLOB visualization_src src/visualization/*(.h|.cpp))
file(GLOB orb2_src third-party/ORB2_SLAM/*(.h|.cpp))

# boost (for logging)
find_package(Boost COMPONENTS log log_setup thread date_time filesystem system REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )

# Loop-detector
file(GLOB loop_detector_src src/loop-closure/*)
add_executable(loop_detection
        ${loop_detector_src}
        ${common_src}
        ${dow2_ext_src}
        ${logging_src}
        ${config_src}
        ${orb2_src})
target_link_libraries( loop_detection ${OpenCV_LIBS}
        ${DBoW2_LIBS}
        ${DLib_LIBS}
        ${Boost_LIBRARIES}
        )

# Vocabulary Converter
add_executable(voc_converter src/tools/VocabularyConverter.cpp ${logging_src} ${dow2_ext_src})
target_link_libraries(
        voc_converter
        ${OpenCV_LIBS}
        ${DBoW2_LIBS}
        ${DLib_LIBS}
        ${Boost_LIBRARIES}
        )

MESSAGE("logging_src: ${logging_src}")

add_executable(view_test src/tests/view_test.cpp
        ${common_src}
        ${logging_src}
        ${visualization_src})
target_link_libraries(view_test ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${DBoW2_LIBS})

#add_executable(play src/play.cpp)
#target_link_libraries(play ${OpenCV_LIBS}
#        ${DBoW2_LIBS}
#        ${DLib_LIBS}
#        ${Boost_LIBRARIES})