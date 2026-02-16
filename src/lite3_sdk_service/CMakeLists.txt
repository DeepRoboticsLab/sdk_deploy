cmake_minimum_required(VERSION 3.10)
project(lite3_sdk_service)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_definitions(-w)
add_compile_options(-fPIC)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drdds REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/service
)

add_executable(sdk_service
  main.cpp
)

ament_target_dependencies(sdk_service
  rclcpp
  drdds
)

install(TARGETS sdk_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

