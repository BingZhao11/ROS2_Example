cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(backward_ros REQUIRED)
find_package(message_filters REQUIRED)

set(CMAKE_BUILD_TYPE "Debug")

find_package(Boost)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

# opencv
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc calib3d)
include_directories(${OpenCV_INCLUDE_DIRS})
MESSAGE(STATUS "OpenCV version is " ${OpenCV_VERSION})


file(GLOB xacro_files urdf/*.urdf.xacro)

foreach(it ${xacro_files})
  # remove .xacro extension
  string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
  set(output_filename ${CMAKE_MATCH_1})
 
  # create a rule to generate ${output_filename} from {it}
  xacro_add_xacro_file(${it} ${output_filename})

  list(APPEND urdf_files ${output_filename})
endforeach(it)
add_custom_target(media_files ALL DEPENDS ${urdf_files})


add_executable(move_calibration_checkerboard src/move_calibration_checkerboard.cpp)
target_include_directories(move_calibration_checkerboard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_calibration_checkerboard PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_calibration_checkerboard
  "xacro"
  "rclcpp"
  "image_transport"
  "geometry_msgs"
  "sensor_msgs"
  "gazebo_msgs"
  "cv_bridge"
  "OpenCV"
  "message_filters"
)

install(TARGETS move_calibration_checkerboard
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch urdf world meshes DESTINATION share/${PROJECT_NAME})


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
