cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)
# Add for WSL Ubuntu
include_directories(/usr/include/opencv4)

add_executable(
    Rasterizer main.cpp 
    rasterizer.hpp rasterizer.cpp
    obj_loader.hpp
    triangle.hpp triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
