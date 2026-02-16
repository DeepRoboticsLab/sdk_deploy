cmake_minimum_required(VERSION 3.10)
project(lite3_transfer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_definitions(-w)
add_compile_options(-fPIC)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drdds REQUIRED)

# ======================
# Platform detection
# ======================
message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# 检测平台架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "x86")
    set(SDK_LIB_NAME "libdeeprobotics_legged_sdk_x86_64.so")
    add_definitions(-DX86_PLATFORM)
    message(STATUS "Platform: x86_64, Library: ${SDK_LIB_NAME}")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(SDK_LIB_NAME "libdeeprobotics_legged_sdk_aarch64.so")
    add_definitions(-DARM_PLATFORM)
    message(STATUS "Platform: ARM/aarch64, Library: ${SDK_LIB_NAME}")
else()
    message(WARNING "Unknown platform: ${CMAKE_SYSTEM_PROCESSOR}, defaulting to x86_64")
    set(SDK_LIB_NAME "libdeeprobotics_legged_sdk_x86_64.so")
    add_definitions(-DX86_PLATFORM)
endif()

# SDK库路径
set(SDK_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/${SDK_LIB_NAME}")

# 检查库文件是否存在
if(NOT EXISTS ${SDK_LIB_PATH})
    message(FATAL_ERROR "
    ===================================================
    SDK library not found!
    Expected path: ${SDK_LIB_PATH}
    Available libraries in ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/ :
    ")
    # 列出可用库
    file(GLOB SDK_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/*.so")
    foreach(lib ${SDK_LIBS})
        message(STATUS "  - ${lib}")
    endforeach()
    message(FATAL_ERROR "===================================================")
endif()

message(STATUS "Using SDK library: ${SDK_LIB_PATH}")

# 头文件包含
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen/
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/gamepad/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Lite3_MotionSDK/include/common
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Lite3_MotionSDK/include
  ${CMAKE_CURRENT_SOURCE_DIR}/interface
  ${CMAKE_CURRENT_SOURCE_DIR}/types
)

# 收集 gamepad 源文件
file(GLOB GAMEPAD_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/gamepad/src/*.cpp"
)

# 创建可执行文件
add_executable(lite3_transfer main.cpp ${GAMEPAD_SOURCES})

# ROS2依赖
ament_target_dependencies(lite3_transfer
  rclcpp
  drdds
)

# 链接库
target_link_libraries(lite3_transfer
  ${SDK_LIB_PATH}
  pthread
  m
  rt
  dl
)

# 安装可执行文件
install(TARGETS lite3_transfer
  DESTINATION lib/${PROJECT_NAME}
)

# 安装SDK库
install(FILES ${SDK_LIB_PATH}
  DESTINATION lib/${PROJECT_NAME}
)

# 设置RPATH
set_target_properties(lite3_transfer PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
  SKIP_BUILD_RPATH FALSE
)

ament_package()