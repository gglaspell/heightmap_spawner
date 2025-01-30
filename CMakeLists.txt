cmake_minimum_required(VERSION 3.8)
project(heightmap_spawner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

add_executable(heightmap_spawner src/heightmap_spawner.cpp)
target_include_directories(heightmap_spawner PUBLIC
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(heightmap_spawner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(heightmap_spawner ${OpenCV_LIBRARIES} gz-transport::core)

ament_target_dependencies(heightmap_spawner rclcpp std_msgs nav_msgs)

install(TARGETS heightmap_spawner
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY example launch textures
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
