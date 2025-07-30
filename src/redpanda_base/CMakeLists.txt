cmake_minimum_required(VERSION 3.5)

project(redpanda_base)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


install(
	DIRECTORY launch urdf meshes config worlds models
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)	
	ament_lint_auto_find_test_dependencies()
endif()
ament_package()