cmake_minimum_required(VERSION 2.8.3)
project(libfovis)

find_package(catkin REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libfovis
)

add_definitions(-Wall -msse2 -msse3)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(fovis
    src/libfovis/frame.cpp
    src/libfovis/visual_odometry.cpp
    src/libfovis/fast.cpp
    src/libfovis/motion_estimation.cpp
    src/libfovis/gauss_pyramid.c
    src/libfovis/refine_motion_estimate.cpp
    src/libfovis/tictoc.cpp
    src/libfovis/primesense_depth.cpp
    src/libfovis/initial_homography_estimation.cpp
    src/libfovis/grid_filter.cpp
    src/libfovis/intensity_descriptor.cpp
    src/libfovis/pyramid_level.cpp
    src/libfovis/feature_matcher.cpp
    src/libfovis/refine_feature_match.cpp
    src/libfovis/stereo_depth.cpp
    src/libfovis/stereo_frame.cpp
    src/libfovis/depth_image.cpp
    src/libfovis/rectification.cpp
    src/libfovis/stereo_rectify.cpp
    src/libfovis/internal_utils.cpp
    src/libfovis/normalize_image.cpp)

set_target_properties(fovis PROPERTIES SOVERSION 1)

install(TARGETS fovis
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h*"
    PATTERN ".git" EXCLUDE
)
