cmake_minimum_required(VERSION 3.27)
cmake_policy(SET CMP0148 OLD)
project(robonaut_telemetry)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Measurements1.msg"
  "msg/Measurements3.msg"
  "msg/Measurements4.msg"
  "msg/Measurements5.msg"
  "msg/Odometry1.msg"
  "msg/Odometry2.msg"
  "msg/Logic1.msg"
  "msg/Logic2.msg"
  "msg/Logic3.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_executable(telemetry_node src/telemetry.cc)
target_include_directories(telemetry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(telemetry_node 
  "rclcpp"  
  "nav_msgs"
  "ament_index_cpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  )

rosidl_target_interfaces(telemetry_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_subdirectory(include/JLB)
include_directories(include/JLB/lib/can_matrix/butl)
include_directories(include/JLB/lib/can_matrix/conf)
include_directories(include/JLB/lib/can_matrix/lib)
target_link_libraries(telemetry_node can_matrix)

install(TARGETS telemetry_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()