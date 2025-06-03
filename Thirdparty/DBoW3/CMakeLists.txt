cmake_minimum_required(VERSION 2.8)
project(DBoW3)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()
set(HDRS_DBOW3
  DBoW3/BowVector.h
  DBoW3/Database.h
  DBoW3/DBoW3.h
  DBoW3/DescManip.h
  DBoW3/dir_reader.h
  DBoW3/exports.h
  DBoW3/FeatureVector.h
  DBoW3/QueryResults.h
  DBoW3/quicklz.h
  DBoW3/timers.h
  DBoW3/ScoringObject.h
  DBoW3/Vocabulary.h)
set(SRCS_DBOW3
  DBoW3/BowVector.cpp
  DBoW3/Database.cpp
  DBoW3/DescManip.cpp
  DBoW3/FeatureVector.cpp
  DBoW3/QueryResults.cpp
  DBoW3/quicklz.c
  DBoW3/ScoringObject.cpp
  DBoW3/Vocabulary.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)


find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.0 not found.")
   endif()
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW3 SHARED ${SRCS_DBOW3} ${SRCS_DUTILS})
target_link_libraries(DBoW3 ${OpenCV_LIBS})

