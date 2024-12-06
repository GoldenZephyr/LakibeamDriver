cmake_minimum_required(VERSION 3.8)
project(lakibeam1)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find_package(pcl_conversions REQUIRED)
#find_package(sensor_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(config_utilities REQUIRED)
#find_package(yaml REQUIRED)
find_package(yaml-cpp REQUIRED)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # comment the line when this package is in a git repo and when
#  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()


include_directories(
curl
)

add_executable(${PROJECT_NAME}_scan_node src/lakibeam1_scan.cpp src/remote.cpp)
target_link_libraries(${PROJECT_NAME}_scan_node curl config_utilities::config_utilities yaml-cpp cnpy)
#target_include_directories(${PROJECT_NAME}_scan_node curl)

#install(TARGETS ${PROJECT_NAME}_scan_node
#  DESTINATION lib/${PROJECT_NAME}
#)
#
#install(DIRECTORY launch rviz
#  DESTINATION share/${PROJECT_NAME}
#)
