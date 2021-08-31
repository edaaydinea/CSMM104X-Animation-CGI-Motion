cmake_minimum_required (VERSION 2.6.2)

project (FOSSSim)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install")

# Initialize the build type (Release, Debug, etc)
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Debug CACHE STRING
    "Choose the type of build, options are: Debug Release."
    FORCE)
endif (NOT CMAKE_BUILD_TYPE)

# TODO: Add checks to ensure that build type is Release or Debug

add_definitions (-DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}")

# Add warnings to the compiler flags
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused")

if (CMAKE_BUILD_TYPE MATCHES Debug)
  add_definitions (-DDEBUG -DEIGEN_DONT_ALIGN -DEIGEN_DONT_VECTORIZE)
endif (CMAKE_BUILD_TYPE MATCHES Debug)

if (CMAKE_BUILD_TYPE MATCHES Release)
  add_definitions (-DNDEBUG)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
endif (CMAKE_BUILD_TYPE MATCHES Release)

# Add directory with macros
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# Add cmake file with useful functions
include (Common)

# Eigen3 library is required
find_package (Eigen3 REQUIRED)
if (EIGEN3_FOUND)
  include_directories (${EIGEN3_INCLUDE_DIR})
else (EIGEN3_FOUND)
  message (SEND_ERROR "Unable to locate Eigen3")
endif (EIGEN3_FOUND)

include_directories (${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory (FOSSSim)
