cmake_minimum_required(VERSION 2.8.3)
project(libfovis)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(catkin REQUIRED cmake_modules)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS libfovis
  LIBRARIES fovis
)

IF(SSE2_FOUND)
    IF(SSSE3_FOUND)
        add_definitions(-Wall -msse2 -msse3)
    ELSE(SSSE3_FOUND)
        add_definitions(-Wall -msse2)
    ENDIF(SSSE3_FOUND)
ELSE(SSE2_FOUND)
    add_definitions(-Wall)
ENDIF(SSE2_FOUND)

include_directories(libfovis
  ${catkin_INCLUDE_DIRS}
)

add_library(fovis
    libfovis/frame.cpp
    libfovis/visual_odometry.cpp
    libfovis/fast.cpp
    libfovis/motion_estimation.cpp
    libfovis/gauss_pyramid.c
    libfovis/refine_motion_estimate.cpp
    libfovis/tictoc.cpp
    libfovis/primesense_depth.cpp
    libfovis/initial_homography_estimation.cpp
    libfovis/grid_filter.cpp
    libfovis/intensity_descriptor.cpp
    libfovis/pyramid_level.cpp
    libfovis/feature_matcher.cpp
    libfovis/refine_feature_match.cpp
    libfovis/stereo_depth.cpp
    libfovis/stereo_frame.cpp
    libfovis/depth_image.cpp
    libfovis/rectification.cpp
    libfovis/stereo_rectify.cpp
    libfovis/stereo_calibration.cpp
    libfovis/internal_utils.cpp
    libfovis/normalize_image.cpp)

set_target_properties(fovis PROPERTIES SOVERSION 1)

install(TARGETS fovis
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h*"
    PATTERN ".git" EXCLUDE
)
