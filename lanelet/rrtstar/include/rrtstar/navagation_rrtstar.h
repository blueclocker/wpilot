/*
 * @Author: wpbit
 * @Date: 2023-04-04 09:13:14
 * @LastEditors: wpbit
 * @LastEditTime: 2023-04-04 16:53:03
 * @Description: 
 */
#ifndef NAVAGATION_RRTSTAR_H_
#define NAVAGATION_RRTSTAR_H_

#include "rrtstar/rrtstar.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>
#include <opencv4/opencv2/opencv.hpp>
#include <osmmap/CarState.h>

namespace RRT
{
class NavagationRRTstar
{
private:
    ros::NodeHandle nh_;
    RRTStar *rrt_plan_;
    vector2d start_point_;
    vector2d goal_point_;
    nav_msgs::OccupancyGrid map_;
    nav_msgs::Path gpspath_;
    std::string map_path_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    double origin_z_;
    int isobs_;
    geometry_msgs::Pose endPose_;
    ros::Publisher path_pub_;
    ros::Publisher nodes_pub_;
    ros::Publisher map_pub_;
    ros::Publisher carstate_pub_;
    ros::Publisher gpspath_pub_;
    ros::Subscriber goal_point_sub_;
    ros::Subscriber localization_sub_;
    std::vector<vector2d> path_;
    bool start_valid_, goal_valid_, map_valid_;
public:
    NavagationRRTstar(ros::NodeHandle &nh);
    ~NavagationRRTstar();
    void run();
    void goalPointCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void localizationCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    void readMap(const std::string &map_path);
    void visualizationPath();
    void visualizationNodes(const std::vector<node2d*> &nodes);
};







}//namespace rrt

#endif