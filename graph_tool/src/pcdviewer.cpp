/*
 * @Author: your name
 * @Date: 2022-01-10 20:32:55
 * @LastEditTime: 2022-01-10 20:45:43
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/pcdviewer.cpp
 */
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
 
int main (int argc, char **argv)
{
  ros::init (argc, argv, "PCDviewer");
  if(argc != 2)
  {
	  ROS_INFO("usage: set point to be delete X" );
	  return 1;
  }
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_out(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 output;
  std::string path;
  pcl::io::loadPCDFile (argv[1], *cloud); //修改自己pcd文件所在路径
  //voxel_filter
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(0.2, 0.2, 0.2);
  filter.filter(*filter_out);
  //Convert the cloud to ROS message
  pcl::toROSMsg(*filter_out, output);
  output.header.frame_id = "map";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer    
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  pcl::io::savePCDFile("/home/wp/wpollo/filter_out.pcd", *filter_out);
  return 0;
}

