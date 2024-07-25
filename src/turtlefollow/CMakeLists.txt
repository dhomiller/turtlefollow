cmake_minimum_required(VERSION 3.5)
project(turtlefollow)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Mode.msg")

set(dependencies "geometry_msgs" "rclcpp" "turtlesim")

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(follow src/follow.cpp)
ament_target_dependencies(follow ${dependencies})
target_link_libraries(follow "${cpp_typesupport_target}")

add_executable(modeswitch src/modeswitch.cpp)
ament_target_dependencies(modeswitch ${dependencies})
target_link_libraries(modeswitch "${cpp_typesupport_target}")

add_executable(mousecontrol src/mousecontrol.cpp)
ament_target_dependencies(mousecontrol ${dependencies})
target_link_libraries(mousecontrol "${cpp_typesupport_target}" xcb)

install(TARGETS follow modeswitch mousecontrol
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
