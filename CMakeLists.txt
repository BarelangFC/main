cmake_minimum_required(VERSION 3.5)
project(main)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bfc_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

list(APPEND CMAKE_MODULE_PATH "/usr/local/share/cmake")
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
pkg_check_modules(ZMQPP REQUIRED libzmqpp)

add_executable(main src/main.cpp)
ament_target_dependencies(main rclcpp std_msgs geometry_msgs std_msgs bfc_msgs sensor_msgs nav_msgs behaviortree_cpp_v3 darknet_ros_msgs nlohmann_json)

include_directories(${ZMQ_INCLUDE_DIRS} ${ZMQPP_INCLUDE_DIRS})
target_link_libraries(main ${ZMQ_LIBRARIES} ${ZMQPP_LIBRARIES})

add_executable(run_lua src/run_lua.cpp)
ament_target_dependencies(run_lua rclcpp bfc_msgs)

# add_executable(tree src/tree.cpp)
# ament_target_dependencies(tree rclcpp std_msgs geometry_msgs std_msgs bfc_msgs sensor_msgs nav_msgs behaviortree_cpp_v3)

install(TARGETS
  # tree
  main
  run_lua
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
