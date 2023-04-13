/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-16 20:39:58
 * @LastEditTime: 2022-09-16 22:35:11
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/rrtstar/src/rrtstar_flow.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "rrtstar/rrtstar_flow.h"

namespace RRT
{
RRTStarflow::RRTStarflow(ros::NodeHandle &nh)
{
    rrt_plan_ = new RRTStar();
    start_point_sub_ = nh.subscribe("/initialpose", 1, &RRTStarflow::startPointCallBack, this);
    goal_point_sub_ = nh.subscribe("/move_base_simple/goal", 1, &RRTStarflow::goalPointCallBack, this);
    map_sub_ = nh.subscribe("/map", 1, &RRTStarflow::mapCallBack, this);//pcdtogm/grid_map
    path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rrtpath", 1);
    nodes_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rrtnodes", 1);
    start_valid_ = false;
    goal_valid_ = false;
    map_valid_ = false;
    has_map_ = false;
    is_plan_ = false;
}

RRTStarflow::~RRTStarflow()
{
    delete rrt_plan_;
}

void RRTStarflow::run()
{
    if(!is_plan_) return;
    if(start_valid_ && goal_valid_ && map_valid_)
    {
        ROS_INFO("rrtstar searching");
        if(rrt_plan_->search(start_point_, goal_point_))
        {
            auto path = rrt_plan_->getPath();
            visualizationPath(path);
            auto nodes = rrt_plan_->getNodes();
            visualizationNodes(nodes);
            rrt_plan_->reset();
            is_plan_ = false;
        }
    }
}

void RRTStarflow::startPointCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    start_point_.x_ = msg->pose.pose.position.x;
    start_point_.y_ = msg->pose.pose.position.y;
    if(rrt_plan_->isPointValid(start_point_)) start_valid_ = true;
    ROS_INFO("get start point");
}

void RRTStarflow::goalPointCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    is_plan_ = true;
    // mutex.lock();
    goal_point_.x_ = msg->pose.position.x;
    goal_point_.y_ = msg->pose.position.y;
    if(rrt_plan_->isPointValid(goal_point_)) goal_valid_ = true;
    ROS_INFO("get goal point");
    // mutex.unlock();
}

void RRTStarflow::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(has_map_) return;
    map_valid_ = true;
    has_map_ = true;
    vector2d originxylow, originxyup;
    vector2i gridsize;
    originxylow.x_ = msg->info.origin.position.x;
    originxylow.y_ = msg->info.origin.position.y;
    originxyup.x_ = originxylow.x_ + msg->info.resolution * msg->info.width;
    originxyup.y_ = originxylow.y_ + msg->info.resolution * msg->info.height;
    gridsize.gridx_ = msg->info.width;
    gridsize.gridy_ = msg->info.height;
    rrt_plan_->init(originxyup, originxylow, gridsize, msg->info.resolution);

    //map_data_输入数据
    // 已知坐标求索引:
    // ix = x/resolution;
    // iy = y/resolution;
    // idx = ix + iy * width;

    // 已知索引求坐标:
    // iy = idx / width;
    // ix = idx % width;
    // x = ix * resolution;
    // y = iy * resolution;
    for(unsigned int i = 0; i < msg->info.height * msg->info.width; ++i)
    {
        if(msg->data[i] != 0)
        {
            vector2d obsp;
            obsp.x_ = i % msg->info.width * msg->info.resolution + msg->info.origin.position.x;
            obsp.y_ = i / msg->info.width * msg->info.resolution + msg->info.origin.position.y;
            rrt_plan_->setObstacle(obsp);
        }
    }
    ROS_INFO("get map");
}

void RRTStarflow::visualizationPath(const std::vector<vector2d> &path)
{
    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker maker;
    maker.header.frame_id = "map";
    maker.header.stamp = ros::Time::now();
    maker.type = visualization_msgs::Marker::LINE_STRIP;
    maker.action = visualization_msgs::Marker::ADD;
    maker.ns = "rrtpath";
    maker.color.a = 1.0;
    maker.color.r = 1.0;
    maker.pose.orientation.w = 1.0;
    maker.scale.x = 0.25;
    maker.id = 1;
    for(int i = 0; i < path.size(); ++i)
    {
        geometry_msgs::Point pos;
        pos.x = path[i].x_;
        pos.y = path[i].y_;
        pos.z = 0;
        maker.points.push_back(pos);
    }
    markerarray.markers.push_back(maker);
    path_pub_.publish(markerarray);
}

void RRTStarflow::visualizationNodes(const std::vector<node2d*> &nodes)
{
    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker maker;
    maker.header.frame_id = "map";
    maker.header.stamp = ros::Time::now();
    maker.type = visualization_msgs::Marker::POINTS;
    maker.action = visualization_msgs::Marker::ADD;
    maker.ns = "rrtnodes";
    maker.color.a = 1.0;
    maker.color.b = 1.0;
    maker.pose.orientation.w = 1.0;
    maker.scale.x = 0.2;
    maker.scale.y = 0.2;
    maker.id = 1;
    for(int i = 0; i < nodes.size(); ++i)
    {
        geometry_msgs::Point pos;
        pos.x = nodes[i]->pxy_.x_;
        pos.y = nodes[i]->pxy_.y_;
        pos.z = 0;
        maker.points.push_back(pos);
    }
    markerarray.markers.push_back(maker);
    nodes_pub_.publish(markerarray);
}



}
