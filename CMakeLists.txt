cmake_minimum_required(VERSION 2.8.3)
project(RRTStar)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_CXX_FLAGS "-march=native -O3 -Wall")

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Eigen3)

if(Eigen3_FOUND)
  set(CUSTOM_EIGEN ${EIGEN3_INCLUDE_DIR})
else()
  find_package(Eigen)
  set(CUSTOM_EIGEN ${EIGEN_INCLUDE_DIR})
endif()

include_directories(
  ${CUSTOM_EIGEN}
)
add_library(RRTStar SHARED src/RRTStar/obstacles.cpp src/RRTStar/rrt.cpp
        src/RRTStar/RRTStarWrapper.cpp)
target_link_libraries(RRTStar Qt5::Widgets Qt5::Gui)