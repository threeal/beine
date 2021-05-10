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
find_package(keisan REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY "include" DESTINATION ".")

add_library(${PROJECT_NAME}
  "src/interface.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} beine_interfaces keisan rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(legs_consumer_log "examples/legs_consumer_log.cpp")
add_executable(legs_provider_dummy "examples/legs_provider_dummy.cpp")
add_executable(stance_simple_filter "examples/stance_simple_filter.cpp")

foreach(target legs_consumer_log legs_provider_dummy stance_simple_filter)
  target_link_libraries(${target} ${PROJECT_NAME})
  install(TARGETS ${target}
    DESTINATION "lib/${PROJECT_NAME}")
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_tests
    "test/compile_test.cpp"
    "test/stance_test.cpp")

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(beine_interfaces keisan rclcpp std_msgs)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})

ament_package()
