cmake_minimum_required(VERSION 3.10)
project(pose_solver)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_common_interfaces REQUIRED)
find_package(dua_geometry_interfaces REQUIRED)
find_package(dua_node_cpp REQUIRED)
find_package(dua_qos_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(params_manager_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simple_serviceclient_cpp REQUIRED)
find_package(std_msgs REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/pose_solver/params.yaml"
  OUT_FILE "init_parameters.cpp")

# Configure Pose Solver node source files
file(GLOB POSE_SOLVER_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/deskew/*.cpp"
  "src/pose_solver/*.cpp")
list(APPEND POSE_SOLVER_SRCS init_parameters.cpp)

# Pose Solver
add_library(pose_solver SHARED ${POSE_SOLVER_SRCS})
target_compile_definitions(pose_solver PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(pose_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(pose_solver
  Eigen3::Eigen)
target_compile_features(pose_solver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pose_solver
  dua_common_interfaces
  dua_geometry_interfaces
  dua_node_cpp
  dua_qos_cpp
  geometry_msgs
  message_filters
  nav_msgs
  params_manager_cpp
  rclcpp
  rclcpp_components
  sensor_msgs
  simple_serviceclient_cpp
  std_msgs)
rclcpp_components_register_nodes(pose_solver "pose_solver::PoseSolver")

# Pose Solver standalone application
add_executable(pose_solver_app
  src/pose_solver_app.cpp)
target_include_directories(pose_solver_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(pose_solver_app PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(pose_solver_app pose_solver)
ament_target_dependencies(pose_solver_app
  dua_app_management
  rclcpp)

# Install Pose Solver component
install(TARGETS pose_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install Pose Solver app
install(TARGETS pose_solver_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
