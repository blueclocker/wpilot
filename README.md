# WPOLLO

#### 相机毫米波雷达融合

##### 毫米波雷达与视觉融合 camera_radar
* 1.打开相机、毫米波雷达（包括kvaser）和YOLO
```
roslaunch camera_radar pre_fusion.launch
```
* 2.启动融合程序
```
roslaunch camera_radar fusion.launch
```

##### 毫米波雷达与视觉外参标定 calibration
* 1.启动相机和毫米波雷达
```
roslaunch calibration calibration_pre.launch
```
* 2.打开ROS动态调参界面
```
roslaunch calibration camera_radar_calibration.launch
```
* 3.保存参数
```
rosparam dump ~/fusion/src/calibration/result/params.yaml
```
* 录包
```
rosbag record -a -x "(.*)/compressed(.*)"
```

##### YOLO darknet_ros
* [darkent_ros](https://github.com/leggedrobotics/darknet_ros)

##### 大恒相机驱动 galaxy_camera
```
roslaunch galaxy_camera MER-231.launch
```

##### 德尔福毫米波雷达驱动 delphi_driver
```
roslaunch kvaser_interface kvaser_can_bridge.launch
```

#### 激光雷达

##### 激光雷达按距离分割点云再使用pcl库欧式聚类 lidar
```
roslaunch lidar test.launch
```

##### autoware提取的激光雷达slam及其组件 liadr_localizer ndt prius_description
* [autoware](https://github.com/Autoware-AI/autoware.ai)
* 小车模型prius_description
* slam
```
roslaunch lidar_localizer ndt_mapping.launch
```

#### 全局路径规划 plan
* 在栅格地图实现A*算法
```
roslaunch plan plan_test.launch
```
* 根据xml文件绘制节点拓扑图发布成ROS的markerarray格式
```
roslaunch plan drawmap.launch
```

#### 工具箱

##### pcd转pgm pcd2pgm
* 读取pcd将其转化为nav_msgs::Occupancy栅格地图，再使用map_server保存
```
roscore
rosrun pcd2pgm pcd2topic
rosrun map_server map_saver -f bitmap
```

##### 多功能ROS工具箱 srv_tools

##### 绘制节点拓扑图 graph_tool
* 根据pcd点云图或栅格地图，手工绘制节点拓扑图
- 启动标定程序，通过rviz 2D Pose Estimate 选定关键点，并自动给出序号
```
roslaunch graph_tool graph_tool.launch
```
* 使用说明：
- 默认开启捕捉模式，捕捉半径在launch文件设置，**searchradius**
- 添加两点连线，勾选add_path
> 在rviz使用**initialpos**，先选定第一个点，在选定第二个点，自动实现连线
- 删除一个点，勾选delete_point
> 在rviz使用**initialpos**，选定要删去的点
- 删除一条线，勾选delete_path
> 与添加连线方法相同

* 注：
- 连线与删除过程，对两点的顺序不敏感
- rviz可能会出现未识别到用户鼠标点击的情况，建议查看终端输出信息，再酌情处理
- 不按照使用说明使用本工具，可能造成无法预知的问题

#### Lanelet

##### adam_shan 改进版 ad_with_lanelet2

##### 高精地图ROS全局规划模块 lanelet
* 启动rviz可视化高精地图以及全局路径规划
```
roslaunch osmmap osmmap.launch
```

#####  串口读取模块  serial
* 打开USB串口权限，如果需要，目前GPS不能从USB读取数据，原因不明
```
sudo chmod 777 /dev/ttyUSB0
rosrun serial serialPort
```

#####  串口数据转标准GPS数据格式模块 odometry_publisher
* 全部GPS/IMU信息，发布fsd_common_msgs::comb, topic = "/comb"
* 里程计信息，发布fsd_common_msgs::Gnss, topic = "/gnss_odom"
* IMU，发布sensor_msgs::Imu, topic = "/gnss_imu"
* ROS标准GPS格式，发布sensor_msgs::NavSatFix, topic = "/gps/fix"
```
rosrun odometry_publishergnss_odom_pub
```

#####  各种自定义msg fsd_common_msgs
* 各种msg，在CmakeList.txt和package.xml设置依赖可以实现跨包调用
- CmakeList.txt
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  fsd_common_msgs
)

add_dependencies(serialPort ${PROJECT_NAME} fsd_common_msgs_gencpp)
```

- package.xml
```
<build_depend>fsd_common_msgs</build_depend>
<exec_depend>fsd_common_msgs</exec_depend>
```

##### 开放空间规划模块 open_space
* 提取自apollo
```
roslaunch open_space demo.launch
```

##### 全局规划跳点法 jps3d
* 取自深蓝学院
```
roslaunch jps3d demo.launch
```

##### 非结构化道路全局规划 rrtstar
* 使用珊格地图
* 支持rrt、rrtstar、A*三种全局规划算法

#### LIO-SAM-6AXIS lio-sam
* 自用激光slam
```
roslaunch lio_sam_6axis run.launch
```

#### ieskf_lio 点云配准定位
* 根据lio-sam的关键帧实时配准定位
```
roslaunch ieskf_lio relocalization.launch
```

#### static_frenet lattice局部规划
* apollo简化版
```
roslaunch static_frenet static_frenet.launch
```

#### 开发版 test
**!!!!!bug多!!!!!**
- 匈牙利算法
- 多线程控制ROS多回调函数
- 等

