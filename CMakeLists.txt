
cmake_minimum_required(VERSION 3.10)
project(AlternativeRoutes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(AlternativeRoutes
    src/main.cpp
    src/graph.hpp
    src/dijkstra.hpp
    src/simulation.hpp
    src/simulation.cpp
)

target_link_libraries(AlternativeRoutes PRIVATE glfw OpenGL::GL)
