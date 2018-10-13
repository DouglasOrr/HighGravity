cmake_minimum_required (VERSION 3.10)
project (high_gravity)

# General settings
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wold-style-cast")

# Programs
add_executable (render src/high_gravity.cpp src/main_render.cpp)
add_executable (tests src/high_gravity.cpp src/main_test.cpp)
target_compile_definitions (tests PRIVATE HIG_TESTING)
