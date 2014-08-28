include_directories(${EIGEN_INCLUDE_DIRS})

add_executable(fv-tum-rgbd main.cpp lodepng.cpp draw.cpp)
target_link_libraries(fv-tum-rgbd fovis z)

pods_install_executables(fv-tum-rgbd)
