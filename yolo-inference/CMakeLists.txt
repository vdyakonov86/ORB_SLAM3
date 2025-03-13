cmake_minimum_required(VERSION 3.23)

option(WITH_OPENCV "build _YOLO_OPENCV" ON)

project(yoloInference LANGUAGES C CXX)

set(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/yolo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp
)
set(HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/yolo.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/utils.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/yolo_classify.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/yolo_detect.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/yolo_segment.h
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_compile_options(-w)
add_compile_options(-D_YOLO_OPENCV)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/opencv SOURCES)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/opencv ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Create the library target
add_library(yoloInference STATIC ${SOURCES} ${HEADERS})

# Add dependencies
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${OpenCV2_INCLUDE_DIRS})
target_link_libraries(yoloInference ${OpenCV_LIBRARIES})



# ------------------------------
# file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# add_compile_options(-w)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Release")

# add_compile_options(-D_YOLO_OPENCV)
# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/opencv SRCS)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/opencv)

# add_executable(yolo ${SRCS})

# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS} ${OpenCV2_INCLUDE_DIRS})
# target_link_libraries(yolo ${OpenCV_LIBRARIES})  
