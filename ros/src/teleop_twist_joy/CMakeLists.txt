cmake_minimum_required(VERSION 3.5)
project(teleop_twist_joy)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME})

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME ${PROJECT_NAME}/${PROJECT_NAME}_export.h)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}")

ament_target_dependencies(${PROJECT_NAME}
  "geometry_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "teleop_twist_joy::TeleopTwistJoy")

add_executable(${PROJECT_NAME}_node src/teleop_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME teleop_node PREFIX "")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_export.h
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Disable some broken tests for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(_teleop_twist_joy_launch_tests
    # Check axes and scaling.
    test/differential_joy_launch_test.py
    test/holonomic_joy_launch_test.py
    test/six_dof_joy_launch_test.py

    # Check enable and turbo button logic.
    test/no_enable_joy_launch_test.py
    test/turbo_enable_joy_launch_test.py
    test/only_turbo_joy_launch_test.py
    test/turbo_angular_enable_joy_launch_test.py

    test/no_require_enable_joy_launch_test.py
  )

  find_package(launch_testing_ament_cmake REQUIRED)
  foreach(_test_path ${_teleop_twist_joy_launch_tests})
    add_launch_test(${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
      TIMEOUT 10
    )
  endforeach()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
