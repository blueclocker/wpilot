/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-16 20:39:32
 * @LastEditTime: 2023-04-04 09:12:06
 * @LastEditors: wpbit
 * @Description: 
 * @FilePath: /catkin_ws/src/rrtstar/include/rrtstar/rrtstar_flow.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef RRTSTAR_FLOW_H_
#define RRTSTAR_FLOW_H_

#include "rrtstar.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>

namespace RRT
{
class RRTStarflow
{
private:
    RRTStar *rrt_plan_;
    vector2d start_point_;
    vector2d goal_point_;
    nav_msgs::OccupancyGrid map_;
    ros::Publisher path_pub_;
    ros::Publisher nodes_pub_;
    ros::Subscriber start_point_sub_;
    ros::Subscriber goal_point_sub_;
    ros::Subscriber map_sub_;
    bool start_valid_, goal_valid_, map_valid_, has_map_, is_plan_;
    std::mutex mutex;
public:
    RRTStarflow(ros::NodeHandle &nh);
    ~RRTStarflow();
    void run();
    void startPointCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void goalPointCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void visualizationPath(const std::vector<vector2d> &path);
    void visualizationNodes(const std::vector<node2d*> &nodes);
};

}

#endif