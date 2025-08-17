cmake_minimum_required(VERSION 3.5)
project(your_custom_bt_node)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)


nav2_package()


include_directories(
  include
)

set(dependencies
  rclcpp
  behaviortree_cpp_v3
  std_msgs
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(nav2_detect_object_condition_bt_node SHARED plugins/condition/detect_object_condition.cpp)
list(APPEND plugin_libs nav2_detect_object_condition_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  target_link_directories(${bt_plugin} PRIVATE "/opt/ros/humble/lib") #add after 19/12/2020 - export LD_LIBRARY_PATH=/home/trsa2024/trsa_lab3/install/your_custom_bt_node/lib:$LD_LIBRARY_PATH

endforeach()

install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)


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

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()



# cmake_minimum_required(VERSION 3.8)
# project(your_custom_bt_node)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(behaviortree_cpp_v3 REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)


# add_library(detect_object_condition_bt_node  SHARED
#   src/detect_object_condition.cpp
# )

# target_include_directories(detect_object_condition_bt_node
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(detect_object_condition_bt_node
#   "rclcpp"
#   "behaviortree_cpp_v3"
#   "std_msgs"
  
# )

# install(
#   TARGETS detect_object_condition_bt_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_export_dependencies(
#   ament_index_cpp
#   behaviortree_cpp_v3
#   nav2_behavior_tree
# )



# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
