#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件

std::string file_directory;
std::string file_name;
std::string pcd_file, pcd_output;

std::string map_topic_name;

const std::string pcd_format = ".pcd";

nav_msgs::OccupancyGrid map_topic_msg;

double thre_z_min = 1.5;
double thre_z_max = 2.0;
int flag_pass_through = 0;

double grid_x = 0.1;
double grid_y = 0.1;
double grid_z = 0.1;

double map_resolution = 0.5;

double thre_radius = 0.1;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);

void PassThroughFilter(const double& thre_low, const double& thre_high, const bool& flag_in);

void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_cloud, const double &radius, const int &thre_count);

int main(int argc, char** argv)
{
   ros::init(argc, argv, "pcl_filters");
   ros::NodeHandle nh;
   ros::NodeHandle private_nh("~");

   ros::Rate loop_rate(1.0);

   private_nh.param("file_directory", file_directory, std::string("/home/wangpeng/wpollo/src/lanelet/osmmap/maps/"));
   ROS_INFO("*** file_directory = %s ***\n", file_directory.c_str());


   private_nh.param("file_name", file_name, std::string("new_linghe"));
   ROS_INFO("*** file_name = %s ***\n", file_name.c_str());

   pcd_file = file_directory + file_name + pcd_format;
   pcd_output = file_directory + "linghe_output" + pcd_format;
   ROS_INFO("*** pcd_file = %s ***\n", pcd_file.c_str());

   private_nh.param("thre_z_min", thre_z_min, 1.5);
   private_nh.param("thre_z_max", thre_z_max, 2.0);
   private_nh.param("flag_pass_through", flag_pass_through, 0);
   private_nh.param("grid_x", grid_x, 0.1);
   private_nh.param("grid_y", grid_y, 0.1);
   private_nh.param("grid_z", grid_z, 0.1);
   private_nh.param("thre_radius", thre_radius, 0.5);
   private_nh.param("map_resolution", map_resolution, 0.5);
   private_nh.param("map_topic_name", map_topic_name, std::string("map"));

   ros::Publisher map_topic_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);

   if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *pcd_cloud) == -1)
   {
     PCL_ERROR ("Couldn't read file: %s \n", pcd_file.c_str());
     return (-1);
   }

   std::cout << "初始点云数据点数：" << pcd_cloud->points.size() << std::endl;

   pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    // build the filter
    sor.setInputCloud (pcd_cloud);
    sor.setLeafSize (0.5, 0.5, 0.5);
    // apply filter
    sor.filter (*filter_cloud);

    pcl::PCDWriter writer;
    writer.write(pcd_output, *filter_cloud);

   return 0;
}

void PassThroughFilter(const double &thre_low, const double &thre_high, const bool &flag_in)
{
    /*方法一：直通滤波器对点云进行处理。*/
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcd_cloud);//输入点云
    passthrough.setFilterFieldName("z");//对z轴进行操作
    passthrough.setFilterLimits(thre_low, thre_high);//设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(flag_in);//true表示保留范围外，false表示保留范围内
    passthrough.filter(*cloud_after_PassThrough);//执行滤波，过滤结果保存在 cloud_after_PassThrough
    std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;
}

void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_cloud0, const double &radius, const int &thre_count)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器

    radiusoutlier.setInputCloud(pcd_cloud0);    //设置输入点云
    radiusoutlier.setRadiusSearch(radius);     //设置radius为100的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(thre_count); //设置查询点的邻域点集数小于2的删除

    radiusoutlier.filter(*cloud_after_Radius);
    std::cout << "半径滤波后点云数据点数：" << cloud_after_Radius->points.size() << std::endl;
}


