cmake_minimum_required(VERSION 3.20)

project(boss)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fdiagnostics-color -O3 -g -march=native)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


# Find required packages
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(LibCMAES REQUIRED)
find_package(NLopt REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(FCL REQUIRED)
find_package(CCD REQUIRED)
# find_package(TBB REQUIRED)



# Set include directories
include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ThirdParty/limbo/src
    /opt/local/include
)



# Create the executable
add_executable(boss boss2.cpp)

# Set precompiled header
target_precompile_headers(boss PRIVATE include/boss_pch.h)

# Link libraries
target_link_libraries(boss PUBLIC
    ${Boost_LIBRARIES}
    Eigen3::Eigen
    LibCMAES::LibCMAES
    NLopt::NLopt
    yaml-cpp::yaml-cpp
    fcl
    ccd
)

# Set library and runtime paths
link_directories(/usr/local/lib /opt/local/lib)

# Set the RPATH
set_target_properties(boss PROPERTIES 
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "/usr/local/lib;/opt/local/lib"
)


