cmake_minimum_required(VERSION 3.8)
project(noether_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ApplyMeshFilters.action"
  "action/GenerateToolPaths.action"
  "action/Segment.action"
  "msg/EigenValueEdgeGeneratorConfig.msg"
  "msg/FilteringConfig.msg"
  "msg/HalfedgeEdgeGeneratorConfig.msg"
  "msg/PlaneSlicerRasterGeneratorConfig.msg"
  "msg/SegmentationConfig.msg"
  "msg/SurfaceWalkRasterGeneratorConfig.msg"
  "msg/ToolPath.msg"
  "msg/ToolPathConfig.msg"
  "msg/ToolPaths.msg"
  "srv/GenerateConvexHull.srv"
  DEPENDENCIES action_msgs geometry_msgs pcl_msgs shape_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
