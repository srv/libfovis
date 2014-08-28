add_definitions(-Wall)

if(USE_SSE)
    add_definitions(-msse2 -msse3)
endif()

#add_definitions(-Wall -march=native -msse2 -msse3 -msse4.2 -g)

add_library(fovis SHARED
    frame.cpp
    visual_odometry.cpp
    fast.cpp
    motion_estimation.cpp
    gauss_pyramid.c
    refine_motion_estimate.cpp
    tictoc.cpp
    primesense_depth.cpp
    initial_homography_estimation.cpp
    grid_filter.cpp
    intensity_descriptor.cpp
    pyramid_level.cpp
    feature_matcher.cpp
    refine_feature_match.cpp
    stereo_calibration.cpp
    stereo_depth.cpp
    stereo_frame.cpp
    depth_image.cpp
    stereo_disparity.cpp
    rectification.cpp
    stereo_rectify.cpp
    internal_utils.cpp
    normalize_image.cpp
    )
set_target_properties(fovis PROPERTIES SOVERSION 1)

pods_install_pkg_config_file(libfovis
    LIBS -lfovis m
    REQUIRES eigen3
    VERSION 0.0.1)

if(PKG_CONFIG_FOUND)
pods_use_pkg_config_packages(fovis eigen3)
endif()

pods_install_libraries(fovis)

pods_install_headers(fovis.hpp
    visual_odometry.hpp
    motion_estimation.hpp
    frame.hpp
    keypoint.hpp
    depth_source.hpp
    camera_intrinsics.hpp
    primesense_depth.hpp
    grid_filter.hpp
    sad.hpp
    internal_utils.hpp
    intensity_descriptor.hpp
    pyramid_level.hpp
    feature_match.hpp
    feature_matcher.hpp
    refine_feature_match.hpp
    stereo_calibration.hpp
    stereo_depth.hpp
    stereo_frame.hpp
    depth_image.hpp
    stereo_disparity.hpp
    rectification.hpp
    stereo_rectify.hpp
    options.hpp
    tictoc.hpp
    refine_motion_estimate.hpp
    initial_homography_estimation.hpp
    DESTINATION fovis
    )
