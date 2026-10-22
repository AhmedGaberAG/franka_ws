cmake_minimum_required(VERSION 3.8)
project(franka_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install the Python package
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/franka_utils/franka_gui
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/franka_gui_pkg)
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/franka_gui_pkg
  DESTINATION lib/${PROJECT_NAME}
)

# Install the entry point
install(
  PROGRAMS franka_utils/franka_gui/main.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_gui_main.py
)

# Create a launcher script at configure time
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/franka_utils/franka_gui/franka_gui.in
  ${CMAKE_CURRENT_BINARY_DIR}/franka_gui
  @ONLY
)
install(
  PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/franka_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()