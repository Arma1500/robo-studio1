cmake_minimum_required(VERSION 3.5.0)
project(map_overlay VERSION 0.1.0 LANGUAGES C CXX)

find_package(OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(map_overlay main.cpp)
target_link_libraries(map_overlay ${OpenCV_LIBS} )

target_include_directories(map_overlay PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})

install(TARGETS map_overlay
  DESTINATION lib/${PROJECT_NAME})


include(CTest)
enable_testing()

