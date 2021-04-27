cmake_minimum_required(VERSION 3.5)

project(beine_cpp)

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
find_package(beine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY "include" DESTINATION ".")

add_executable(legs_consumer_log "bin/legs_consumer_log.cpp")
add_executable(legs_provider_dummy "bin/legs_provider_dummy.cpp")
add_executable(stance_simple_filter "bin/stance_simple_filter.cpp")

foreach(target legs_consumer_log legs_provider_dummy stance_simple_filter)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(${target}
    beine_interfaces rclcpp std_msgs)

  install(TARGETS ${target}
    DESTINATION "lib/${PROJECT_NAME}")
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_tests
    "test/compile_test.cpp"
    "test/stance_test.cpp")

  target_include_directories(${PROJECT_NAME}_tests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(${PROJECT_NAME}_tests
    beine_interfaces rclcpp std_msgs)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(beine_interfaces rclcpp std_msgs)
ament_export_include_directories("include")

ament_package()
