cmake_minimum_required(VERSION 3.10)

project(s10_sdk_deploy)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# 警告忽略和编译选项
add_definitions(-w)
add_compile_options(-fPIC -pipe)

# 配置选项
set(BUILD_PLATFORM "x86" CACHE STRING "Target platform (x86 or arm)")

if(BUILD_PLATFORM STREQUAL "arm")
  set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drdds REQUIRED)

set(THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

include_directories(
  include/types
  include/utils
  interface/robot
  interface/user_command
  interface/network
  state_machine
  run_policy
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/include
)

# 运行时路径设置
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


set(CMAKE_INSTALL_RPATH 
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/
  ../lib/onnxruntime/
)

file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")
file(GLOB_RECURSE GAMEPAD_SDK_SRC "${THIRD_PARTY}/gamepad/src/*.cpp")

add_executable(rl_deploy
  main.cpp
  ${STATE_MACHINE_SRC}
  ${GAMEPAD_SDK_SRC}
  interface/robot/hardware/dds_interface.cpp
  interface/user_command/dds_command_interface.cpp
  interface/user_command/gamepad_interface.cpp
  run_policy/s10_policy_params.cpp
)

set_source_files_properties(main.cpp PROPERTIES COMPILE_OPTIONS "-O1")
set_source_files_properties(interface/robot/hardware/dds_interface.cpp PROPERTIES COMPILE_OPTIONS "-O0")
set_source_files_properties(interface/user_command/dds_command_interface.cpp PROPERTIES COMPILE_OPTIONS "-O0")
set_source_files_properties(interface/user_command/gamepad_interface.cpp PROPERTIES COMPILE_OPTIONS "-O0")

set_target_properties(rl_deploy PROPERTIES UNITY_BUILD ON UNITY_BUILD_BATCH_SIZE 0)
set_source_files_properties(
  main.cpp
  state_machine/parameters/s10_control_parameters.cpp
  interface/robot/hardware/dds_interface.cpp
  interface/user_command/dds_command_interface.cpp
  interface/user_command/gamepad_interface.cpp
  run_policy/s10_policy_params.cpp
  PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON
)

if(NOT BUILD_PLATFORM STREQUAL "arm")
  find_program(S10_LLD ld.lld)
  if(S10_LLD)
    target_link_options(rl_deploy PRIVATE -fuse-ld=lld)
  endif()
endif()

add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${THIRD_PARTY}/onnxruntime//${BUILD_PLATFORM}/lib/libonnxruntime.so")

ament_target_dependencies(rl_deploy
  rclcpp
  std_msgs
  drdds
)
if(BUILD_PLATFORM STREQUAL "arm")
  target_link_libraries(rl_deploy
    onnxruntime
    -lpthread
    -lm
    -lrt
    -ldl
    -lstdc++
  )
else()
  target_link_libraries(rl_deploy
    onnxruntime
    -lpthread
    -lm
    -lrt
    -ldl
    -lstdc++
  )
endif()


install(TARGETS rl_deploy
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()