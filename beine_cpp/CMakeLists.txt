cmake_minimum_required(VERSION 3.5)

project(beine_cpp)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(argparse REQUIRED)
find_package(beine_interfaces REQUIRED)
find_package(keisan REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY "include" DESTINATION ".")

add_library(${PROJECT_NAME} SHARED
  "src/consumer/joints_consumer.cpp"
  "src/consumer/legs_consumer.cpp"
  "src/node/legs_node.cpp"
  "src/provider/legs_provider.cpp"
  "src/provider/stance_provider.cpp"
  "src/utility/interface.cpp"
  "src/utility/stance.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  argparse beine_interfaces keisan rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(legs_logger "examples/legs_logger.cpp")
add_executable(legs_teleop "examples/legs_teleop.cpp")
add_executable(simple_stance_filter "examples/simple_stance_filter.cpp")

foreach(target legs_logger legs_teleop simple_stance_filter)
  target_link_libraries(${target} ${PROJECT_NAME})
  install(TARGETS ${target}
    DESTINATION "lib/${PROJECT_NAME}")
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_tests
    "test/stance_test.cpp")

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(argparse beine_interfaces keisan rclcpp std_msgs)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})

ament_package()
