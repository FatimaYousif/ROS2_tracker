cmake_minimum_required(VERSION 3.8)
project(tracking_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------
add_executable(main src/main.cpp )
ament_target_dependencies(main rclcpp vision_msgs yolo_msgs)
install(TARGETS main DESTINATION lib/${PROJECT_NAME})

# another exec 2
# add_executable(follow_target src/follow_target.cpp)
# ament_target_dependencies(
#   follow_target
#   rclcpp
#   vision_msgs
#   yolo_msgs
#   px4_ros2_cpp
#   px4_msgs
# )
# target_include_directories(follow_target PRIVATE ${EIGEN3_INCLUDE_DIRS})
# install(TARGETS follow_target DESTINATION lib/${PROJECT_NAME})


# exec 3
# add_executable(exec_follow_target src/exec_follow_target.cpp)
# ament_target_dependencies(
#   exec_follow_target
#   rclcpp
#   vision_msgs
#   yolo_msgs
#   px4_ros2_cpp
#   px4_msgs
# )
# target_include_directories(exec_follow_target PRIVATE ${EIGEN3_INCLUDE_DIRS})
# install(TARGETS exec_follow_target DESTINATION lib/${PROJECT_NAME})


# exec 4 
add_executable(counter src/counter.cpp)
ament_target_dependencies(
  counter
  rclcpp
  vision_msgs
  yolo_msgs
  px4_ros2_cpp
  px4_msgs
)
target_include_directories(counter PRIVATE ${EIGEN3_INCLUDE_DIRS})
install(TARGETS counter DESTINATION lib/${PROJECT_NAME})



# exec 5
# add_executable(mot src/mot.cpp)
# ament_target_dependencies(
#   mot
#   rclcpp
#   vision_msgs
#   yolo_msgs
#   px4_ros2_cpp
#   px4_msgs
# )
# target_include_directories(mot PRIVATE ${EIGEN3_INCLUDE_DIRS})
# install(TARGETS mot DESTINATION lib/${PROJECT_NAME})


# exec 6
add_executable(search_only src/search_only.cpp)
ament_target_dependencies(
  search_only
  rclcpp
  vision_msgs
  yolo_msgs
  px4_ros2_cpp
  px4_msgs
)
target_include_directories(search_only PRIVATE ${EIGEN3_INCLUDE_DIRS})
install(TARGETS search_only DESTINATION lib/${PROJECT_NAME})




# Install Python launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()

