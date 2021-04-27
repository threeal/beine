cmake_minimum_required(VERSION 3.5)

project(beine_interfaces)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Joints.msg"
  "msg/Orientation.msg"
  "msg/Position.msg"
  "msg/Stance.msg")

install(DIRECTORY "include" DESTINATION ".")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_tests
    "test/compile_test.cpp")

  target_include_directories(${PROJECT_NAME}_tests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  rosidl_target_interfaces(${PROJECT_NAME}_tests
    ${PROJECT_NAME} "rosidl_typesupport_cpp")

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories("include")

ament_package()
