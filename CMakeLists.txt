cmake_minimum_required(VERSION 3.10)
project(KalmanFilterProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add executable
add_executable(main main.cpp)

# Link libraries (Eigen is a header-only library, so no linking is needed)
target_include_directories(main PRIVATE ${EIGEN3_INCLUDE_DIR})
