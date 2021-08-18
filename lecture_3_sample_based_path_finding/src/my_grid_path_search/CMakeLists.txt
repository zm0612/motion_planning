cmake_minimum_required(VERSION 3.0.2)
project(my_grid_path_search)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_executable(generate_random_map
    app/generate_random_map.cpp
)
target_link_libraries(generate_random_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)