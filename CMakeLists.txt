cmake_minimum_required(VERSION 3.20)
project(ompl_bow)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -fdiagnostics-color -O3 -g -march=native)
endif()


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
add_executable(ompl_bow ompl_bow.cpp
        include/pch.h
        src/MotionPlanner.cpp
        include/MotionPlanner.h
        src/CboDirectedControlSampler.cpp
        include/CboDirectedControlSampler.h
        src/KoulesControlSpace.cpp
        include/Koules/KoulesControlSpace.h
        include/Koules/KoulesConfig.h
        include/Koules/KoulesStateSpace.h
        src/KoulesStateSpace.cpp
        include/Koules/KoulesProjection.h
)

# Set precompiled header
target_precompile_headers(ompl_bow PRIVATE include/pch.h)

# Link libraries
target_link_libraries(ompl_bow PUBLIC
    ${Boost_LIBRARIES}
    Eigen3::Eigen
    -lompl
    yaml-cpp::yaml-cpp
    fcl
    ccd
)