cmake_minimum_required(VERSION 3.5)
project(drdds)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-w -pipe)
endif()

set(BUILD_PLATFORM "x86" CACHE STRING "Target platform (x86 or arm)")
if(NOT BUILD_PLATFORM STREQUAL "arm")
  find_program(DRDDS_LLD ld.lld)
  if(DRDDS_LLD)
    add_link_options(-fuse-ld=lld)
  endif()
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Skip unused Rust generator everywhere and Python bindings on ARM target.
list(FILTER AMENT_EXTENSIONS_rosidl_generate_idl_interfaces EXCLUDE REGEX "rosidl_generator_rs")
if(BUILD_PLATFORM STREQUAL "arm")
  list(FILTER AMENT_EXTENSIONS_rosidl_generate_idl_interfaces EXCLUDE REGEX "rosidl_generator_py|pyext")
else()
  # Allow drdds__py codegen to run in parallel with C++ typesupport (it only depends on .idl files).
  function(add_custom_command)
    set(_is_py FALSE)
    math(EXPR _last "${ARGC} - 1")
    foreach(_i RANGE 0 ${_last})
      if("${ARGV${_i}}" STREQUAL "Generating Python code for ROS interfaces")
        set(_is_py TRUE)
      endif()
    endforeach()
    if(NOT _is_py)
      _add_custom_command(${ARGN})
      return()
    endif()
    set(_eval "_add_custom_command(")
    foreach(_i RANGE 0 ${_last})
      if(NOT "${ARGV${_i}}" STREQUAL "${PROJECT_NAME}")
        string(APPEND _eval " [=[${ARGV${_i}}]=]")
      endif()
    endforeach()
    string(APPEND _eval ")")
    cmake_language(EVAL CODE "${_eval}")
  endfunction()
endif()

file(GLOB_RECURSE MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "msg/*.msg")
list(TRANSFORM MSG_FILES PREPEND "msg/")

file(GLOB_RECURSE SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv" "srv/*.srv")
list(TRANSFORM SRV_FILES PREPEND "srv/")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

# Batch generated C/C++ translation units to cut compiler launch and header parsing overhead.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.16")
  foreach(_drdds_tgt
    ${PROJECT_NAME}__rosidl_generator_c
    ${PROJECT_NAME}__rosidl_typesupport_fastrtps_c
    ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
    ${PROJECT_NAME}__rosidl_typesupport_introspection_c
    ${PROJECT_NAME}__rosidl_typesupport_c
    ${PROJECT_NAME}__rosidl_typesupport_cpp
    ${PROJECT_NAME}__rosidl_generator_py
    ${PROJECT_NAME}__rosidl_typesupport_fastrtps_c__pyext
    ${PROJECT_NAME}__rosidl_typesupport_introspection_c__pyext
    ${PROJECT_NAME}__rosidl_typesupport_c__pyext
  )
    if(TARGET ${_drdds_tgt})
      set_target_properties(${_drdds_tgt} PROPERTIES
        UNITY_BUILD ON
        UNITY_BUILD_BATCH_SIZE 0
      )
    endif()
  endforeach()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

