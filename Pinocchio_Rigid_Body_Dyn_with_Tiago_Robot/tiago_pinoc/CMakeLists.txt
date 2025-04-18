cmake_minimum_required(VERSION 3.8)
project(tiago_pinoc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pinocchio REQUIRED)
find_package(PythonLibs REQUIRED)

add_executable(my_cpp_node src/my_cpp_node.cpp)
target_include_directories(my_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(my_cpp_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  my_cpp_node
  "rclcpp"
  "rclpy"
)


install(TARGETS my_cpp_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python script
install(PROGRAMS
  scripts/my_pinoccohio_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# target_link_libraries(my_cpp_node pinocchio::pinocchio)
# add_library(parsers SHARED ${SOURCE_FILES})
# include_directories(${PYTHON_INCLUDE_DIRS})
# target_link_libraries(parsers ${PYTHON_LIBRARIES})

target_link_libraries(my_cpp_node pinocchio::pinocchio)

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
