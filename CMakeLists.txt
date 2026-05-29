cmake_minimum_required(VERSION 3.14)
project(teleop_hand_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(igris_sdk_DIR "${CMAKE_CURRENT_SOURCE_DIR}/igris_sdk_robot/lib/cmake/igris_sdk")
find_package(igris_sdk REQUIRED)

add_executable(quest_handcmd_bridge quest_handcmd_bridge.cpp)
ament_target_dependencies(quest_handcmd_bridge rclcpp std_msgs)
target_link_libraries(quest_handcmd_bridge igris_sdk::igris_sdk)

set_target_properties(
  quest_handcmd_bridge PROPERTIES
  INSTALL_RPATH "$ORIGIN;${CMAKE_CURRENT_SOURCE_DIR}/igris_sdk_robot/lib;${CMAKE_CURRENT_SOURCE_DIR}/igris_sdk_robot/thirdparty/lib"
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/igris_sdk_robot/lib;${CMAKE_CURRENT_SOURCE_DIR}/igris_sdk_robot/thirdparty/lib"
)

install(TARGETS quest_handcmd_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
