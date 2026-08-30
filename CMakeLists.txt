cmake_minimum_required(VERSION 3.23)

project(
  gz_betaflight_bridge
  VERSION 0.1.1
  
  DESCRIPTION "Standalone Betaflight SITL to Gazebo Sim bridge"
  LANGUAGES CXX
)

include(CTest)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GZ_TRANSPORT REQUIRED IMPORTED_TARGET gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED IMPORTED_TARGET gz-msgs10)
pkg_check_modules(GZ_MATH REQUIRED IMPORTED_TARGET gz-math7)
pkg_check_modules(YAML_CPP REQUIRED IMPORTED_TARGET yaml-cpp)
pkg_check_modules(SPDLOG REQUIRED IMPORTED_TARGET spdlog)

configure_file(
  cmake/Version.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/betaflight_gazebo_bridge/Version.hh
  @ONLY
)

add_library(bridge_core
  src/BridgeApp.cc
  src/Config.cc
  src/GazeboTransport.cc
  src/MotorMapper.cc
  src/UdpSocket.cc
)

target_include_directories(bridge_core
  PUBLIC
    include
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)
target_compile_features(bridge_core PUBLIC cxx_std_20)
target_link_libraries(bridge_core
  PUBLIC
    PkgConfig::GZ_TRANSPORT
    PkgConfig::GZ_MSGS
    PkgConfig::GZ_MATH
    PkgConfig::YAML_CPP
    PkgConfig::SPDLOG
)

add_executable(betaflight_gazebo_bridge src/main.cpp)
target_link_libraries(betaflight_gazebo_bridge PRIVATE bridge_core)
add_custom_command(TARGET betaflight_gazebo_bridge POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_SOURCE_DIR}/config/bridge.yaml
    $<TARGET_FILE_DIR:betaflight_gazebo_bridge>/bridge.yaml
)

add_executable(test_bridge_core test/test_bridge_core.cpp)
target_link_libraries(test_bridge_core PRIVATE bridge_core)
add_test(NAME bridge_core COMMAND test_bridge_core)

foreach(target bridge_core betaflight_gazebo_bridge test_bridge_core)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /WX)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endforeach()
