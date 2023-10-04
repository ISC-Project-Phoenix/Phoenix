cmake_minimum_required(VERSION 3.8)
project(phoenix_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(phoenix_description REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(robot_state_controller REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(joy REQUIRED)
find_package(logi_g29 REQUIRED)
find_package(teleop_twist_joy REQUIRED)
find_package(gz_io_ros REQUIRED)
find_package(data_logger REQUIRED)
find_package(twist_to_ackermann REQUIRED)

find_package(obj_detector REQUIRED)
find_package(obj_planner REQUIRED)
find_package(obj_tracker REQUIRED)
find_package(hybrid_pp REQUIRED)

#INSTALL
install(
        DIRECTORY launch worlds models config
        DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
