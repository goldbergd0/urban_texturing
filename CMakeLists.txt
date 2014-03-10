project (URBAN_TEXTURE)

cmake_minimum_required (VERSION 2.6)

#set(LIB_MACROS "")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
         "${URBAN_TEXTURE_SRC_DIR}/cmake")

#set(Eigen3_DIR  

# Find external libraries
find_package(Eigen3 REQUIRED)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

string(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type_tolower)
if(    NOT cmake_build_type_tolower STREQUAL "debug"
   AND NOT cmake_build_type_tolower STREQUAL "release")
  message(FATAL_ERROR "Unknown build type \"${CMAKE_BUILD_TYPE}\". Allowed values are Debug, Release (case-insensitive).")
endif()

#include_directories(${IMMERSIVE_GAMING_SOURCE_DIR})
#include_directories(${IMMERSIVE_GAMING_BINARY_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "-Wall -g")
if (cmake_build_type_tolower STREQUAL "release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
endif()

file(GLOB_RECURSE HEADERS "*/*.h" "*.h")
file(GLOB_RECURSE SOURCES "*/*.cpp" "*.cpp")

add_library(urban_texture ${SOURCES} ${HEADERS})

add_executable(main main.cpp)
target_link_libraries(main urban_texture)
#target_link_libraries(illumipong ${GLUT_LIBRARY})
#target_link_libraries(illumipong ${OPENGL_LIBRARIES})
#target_link_libraries(illumipong ${OpenCV_LIBRARIES}
