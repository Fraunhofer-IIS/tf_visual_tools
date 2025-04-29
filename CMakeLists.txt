cmake_minimum_required(VERSION 3.8)
project(tf_visual_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/tf_visual_tools/tf_visual_tools_gui.hpp
)

add_library(tf_visual_tools_gui 
  src/tf_visual_tools_gui.cpp 
  src/gui_remote_receiver.cpp
  src/tf_manager.cpp
  src/tf_publisher.cpp 
  ${MOC_FILES})

target_include_directories(tf_visual_tools_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tf_visual_tools_gui
  pluginlib
  rviz_common
  visualization_msgs
  interactive_markers
  tf2
  tf2_ros
  rclcpp
)
install(TARGETS tf_visual_tools_gui
        EXPORT export_tf_visual_tools
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
add_executable(tf_publisher_node src/tf_publisher.cpp src/tf_manager.cpp)
ament_target_dependencies(tf_publisher_node
  rclcpp
  tf2
  tf2_ros
  visualization_msgs
  std_msgs
)
target_link_libraries(tf_publisher_node tf_visual_tools_gui)
install(TARGETS tf_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES plugin_description.xml
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY 
            launch
            config
        DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_targets(export_tf_visual_tools)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

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
