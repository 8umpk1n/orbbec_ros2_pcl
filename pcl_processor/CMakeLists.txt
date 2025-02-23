cmake_minimum_required(VERSION 3.8)
project(pcl_processor)

# 设置CMake策略（解决CMP0074警告）
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

# 关键修改：显式查找所有依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)  # 必须单独声明
find_package(PCL REQUIRED COMPONENTS common io filters)  # 添加关键组件


add_executable(pcl_process src/pcl_processor.cpp)
ament_target_dependencies(pcl_process
  rclcpp
  sensor_msgs
  pcl_conversions
)

# 修正链接顺序和目标名称
target_link_libraries(pcl_process
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES} # 使用命名空间目标
  ${PCL_LIBRARIES}  # 放在最后
)
install(TARGETS
  pcl_process 
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

