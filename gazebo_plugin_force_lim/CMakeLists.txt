cmake_minimum_required(VERSION 3.5)
project(gazebo_plugin_force_lim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(gazebo REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})



# gazebo_ros_force
add_library(gazebo_ros_force_lim SHARED src/gazebo_ros_force_lim.cpp)
target_include_directories(gazebo_ros_force_lim PUBLIC include)
ament_target_dependencies(gazebo_ros_force_lim
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_force_lim)

target_include_directories(gazebo_ros_force_lim PUBLIC ${ignition-common3_INCLUDE_DIRS})
target_link_libraries(gazebo_ros_force_lim ${ignition-common3_LIBRARIES})




# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY env_hooks
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.sh.in")

install(TARGETS
  gazebo_ros_force_lim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
