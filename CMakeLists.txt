cmake_minimum_required(VERSION 3.20)

project(aimpoint)

set(CMAKE_CXX_STANDARD 17)

#set(BUILD_SHARED_LIBS OFF)
add_subdirectory("deps/glad")

add_executable(aimpoint main.cpp)

target_link_libraries(
        aimpoint
        glad
)