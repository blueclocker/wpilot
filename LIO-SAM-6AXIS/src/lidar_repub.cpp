/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-08-24 14:15:32
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-09-02 20:55:11
 */

#include "../include/utility.h"
struct VelodynePointXYZIRT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

void MySigintHandler(int sig)
{
  //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
  ROS_INFO("lidar_repub shutting down!");
  ros::shutdown();
}

class lidarRepub
{
public:
  ros::Publisher pub_pcl;
  ros::Subscriber sub;
  ros::NodeHandle nh;
  ros::Subscriber imu_calibration_sub;
  bool calib_flag = false;
  Eigen::Vector3f extTrans{-1.0510799, 0, -1.96};
  lidarRepub()
  {
    sub = nh.subscribe("points_raw", 1, &lidarRepub::LidarCbk, this);
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_calib", 1);
  }

  void LidarCbk(const sensor_msgs::PointCloud2ConstPtr &msg)
  {

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserIn(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<VelodynePointXYZIRT>::Ptr outCloud(new pcl::PointCloud<VelodynePointXYZIRT>());
    pcl::fromROSMsg(*msg, *laserIn);

    for (size_t i = 0; i < laserIn->size(); ++i)
    {

      VelodynePointXYZIRT point;
      Eigen::Vector3f origPoint{laserIn->points[i].x, laserIn->points[i].y, laserIn->points[i].z};
      Eigen::Vector3f basePoint = origPoint + extTrans;
      point.x = basePoint.x();
      point.y = basePoint.y();
      point.z = basePoint.z();
      point.intensity = i % 100 + 30;
      point.time = 0;
      point.ring = i % 32;
      outCloud->push_back(point);
    }

    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*outCloud, outMsg);
    outMsg.header = msg->header;
    outMsg.header.frame_id = "base_link";
    pub_pcl.publish(outMsg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_repub");
  ros::NodeHandle nh;
  ROS_INFO("\033[1;32m----> start lidarRepub. \033[0m");
  lidarRepub lidar_repub;
  signal(SIGINT, MySigintHandler);
  ros::spin();
  return 0;
}
