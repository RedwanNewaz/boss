cmake_minimum_required(VERSION 3.20)
project(ompl_bow)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(FCL REQUIRED)
find_package(CCD REQUIRED)

# Set include directories
include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
    /opt/local/include
    /usr/local/include/ompl-1.6
)

link_directories(/usr/local/lib)

# Create the executable
add_executable(ompl_bow ompl_bow.cpp)

# Set precompiled header


# Link libraries
target_link_libraries(ompl_bow PUBLIC
    ${Boost_LIBRARIES}
    Eigen3::Eigen
    -lompl
    yaml-cpp::yaml-cpp
    fcl
    ccd
)