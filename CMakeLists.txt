cmake_minimum_required(VERSION 3.8)
project(sensingrigs_communicator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# === Message and Service Definitions ===
set(msg_files
  "msg/MonoIR.msg"
  "msg/Odometry.msg"
  "msg/StereoIR.msg"
)

set(srv_files
  "srv/DataQuery.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

# === Include paths ===
include_directories(
  include
)


#add_executable(client
#  src/client.cpp
#  src/ServiceHandler.cpp
#)



rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#add_dependencies(client ${cpp_typesupport_target})

#install(TARGETS client
#  DESTINATION lib/${PROJECT_NAME}
#)
# === CLIENT ===
add_executable(client src/client.cpp)
ament_target_dependencies(client
 rclcpp
)
target_include_directories(
  client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)
add_dependencies(client ${cpp_typesupport_target})

target_link_libraries(client
  ${cpp_typesupport_target}
)


# === SERVICE ===
add_executable(service
  src/service.cpp
  # src/FileReader.cpp
  # src/Publisher.cpp
  src/ServiceHandler.cpp
)


ament_target_dependencies(service
  rclcpp
  std_msgs
)

target_include_directories(
  service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

add_dependencies(service ${cpp_typesupport_target})

target_link_libraries(service
  ${cpp_typesupport_target}
)

install(TARGETS service client
  DESTINATION lib/${PROJECT_NAME}
)

# === Install headers ===
install(DIRECTORY include/
  DESTINATION include/
)

# === Export interfaces ===
ament_export_dependencies(rosidl_default_runtime)
ament_package()
