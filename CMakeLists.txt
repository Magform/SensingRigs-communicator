cmake_minimum_required(VERSION 3.8)
project(SensingRigs_communicator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(SDL2 REQUIRED)
find_package(yaml-cpp REQUIRED)


include_directories(
  include
  ${SDL2_INCLUDE_DIRS}
)

# _______CLIENT_______
add_executable(client
  src/client.cpp
  src/ServiceHandler.cpp
)

ament_target_dependencies(client
  rclcpp
  std_msgs
)

target_link_libraries(client
  ${SDL2_LIBRARIES}
  yaml-cpp
)

install(TARGETS client
  DESTINATION lib/${PROJECT_NAME}
)

# _______SERVICE_______
add_executable(service
  src/service.cpp
  src/FileReader.cpp
  src/Publisher.cpp
  src/ServiceHandler.cpp
)

ament_target_dependencies(service
  rclcpp
  std_msgs
)

target_link_libraries(service
  ${SDL2_LIBRARIES}
  yaml-cpp
)

install(TARGETS service
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()