# orbbec_ros2_pcl
Orbbec 3D Camera    绿色植物背景分离
  需下载Orbbec_ros2
   #需修改gemini_330_series.launch.py中enable_colored_point_cloud为true
  1.启动相机节点
  ```bash
  ros2 launch orbbec_camera  gemini_330_series.launch.py 

  2.编译文件
  ```bash
  cd pointcloud_subscriber
  colcon build --packages-select pointcloud_subscriber
  chmod +x install/setup.bash
  source install/setup.bash

  3.修改config/params.yaml文件
    依据实际情况进行修改/范围/颜色/聚类/间隔时间
      
  4.运行背景分离节点
  ```bash
  ros2 launch pointcloud_subscriber pointcloud_processor.launch.py 
  
  结果自动保存在pointcloud_subscriber文件夹下，为*.pcd文件，*.las文件。final.las为最终结果
  


