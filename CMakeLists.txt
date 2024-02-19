cmake_minimum_required(VERSION 3.5)
project(obu)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_export_include_directories(include/obu)

ament_auto_add_executable(${PROJECT_NAME}_server_node
  src/udp_server.cpp
  src/obu_socket_node.cpp
  src/data_process.cpp

)

# install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  include
  # config
)
