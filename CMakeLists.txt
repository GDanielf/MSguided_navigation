cmake_minimum_required(VERSION 3.8)
project(guided_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(message_filters REQUIRED)
install(
  PROGRAMS
    src/multi_camera.py
    src/triangulation.py
    src/stereo_camera.py
    src/filtro_particulas.py
    src/compare_position.py
    src/navigation.py
    src/planner.py
    src/mapa.py
    src/particle.py
  DESTINATION lib/${PROJECT_NAME}  
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImagesAngles.msg"
  "msg/PoseEstimate.msg"
  DEPENDENCIES sensor_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
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
