cmake_minimum_required(VERSION 3.16)
project(gst_gzimgsrc VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-base-1.0
  gstreamer-video-1.0
)

find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(gstgzimgsrc SHARED
  src/gstgzimgsrc.cpp
)

target_compile_options(gstgzimgsrc PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(gstgzimgsrc PRIVATE PACKAGE="gst_gzimgsrc")
target_link_libraries(gstgzimgsrc
  PRIVATE
    PkgConfig::GSTREAMER
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
)

set_target_properties(gstgzimgsrc PROPERTIES
  PREFIX ""
  OUTPUT_NAME "libgstgzimgsrc"
)

install(TARGETS gstgzimgsrc
  LIBRARY DESTINATION lib/gstreamer-1.0
)
