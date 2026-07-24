cmake_minimum_required(VERSION 3.23)

project(
  gz_betaflight_bridge
  VERSION 0.1.0
  DESCRIPTION "Betaflight SITL to Gazebo Sim bridge bootstrap"
  LANGUAGES CXX
)

add_executable(hello_bridge src/main.cpp)

target_compile_features(hello_bridge PRIVATE cxx_std_20)

if(MSVC)
  target_compile_options(hello_bridge PRIVATE /W4 /WX)
else()
  target_compile_options(hello_bridge PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

