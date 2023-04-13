/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-16 20:39:47
 * @LastEditTime: 2022-09-16 23:30:55
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/rrtstar/src/rrtstar.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "rrtstar/rrtstar.h"
#include <chrono>
#include <random>

namespace RRT
{
RRTStar::~RRTStar()
{
    if(map_data_ != nullptr)
    {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    delete goal_node_;
    goal_node_ = nullptr;

    for(unsigned int i = 0; i < nodes_.size(); ++i)
    {
        if(nodes_[i] != nullptr)
        {
            delete nodes_[i];
            nodes_[i] = nullptr;
        }
    }
    nodes_.clear();
}

bool RRTStar::search(const vector2d &start, const vector2d &goal)
{
    vector2d startpt = checkPoint(start);
    vector2d goalpt = checkPoint(goal);

    goal_node_ = new node2d(goalpt);
    start_node_ = new node2d(startpt);
    nodes_.emplace_back(start_node_);

    pcl::PointCloud<pcl::PointXY> pointcloudnode = getNodesCloudPoint();
    kdtree_flann_.setInputCloud(pointcloudnode.makeShared());

    //开始
    std::chrono::time_point<std::chrono::system_clock> starttime, endtime;
    starttime = std::chrono::system_clock::now();
    while(true)
    {
        vector2d rand_point = sample();
        node2d* near_node_ptr = near(rand_point);

        vector2d new_point = steer(rand_point, near_node_ptr->pxy_, resolution_*2);
        // std::cout << "new point pos: " << new_point.x_ << " ," << new_point.y_ << std::endl;

        //新点越界
        if(new_point.x_ < pxy_lower_.x_ || new_point.x_ >= pxy_upper_.x_ ||
           new_point.y_ < pxy_lower_.y_ || new_point.y_ >= pxy_upper_.y_) continue;

        //碰撞检测
        if(!collisionFree(near_node_ptr->pxy_, new_point)) continue;

        node2d *new_node_ptr = new node2d(new_point);
        std::vector<node2d*> near_nodes;
        near_nodes = nearC(new_point, resolution_ * 4);
        chooseParent(near_nodes, new_node_ptr);
        rewire(near_nodes, new_node_ptr);

        //采样点距离目标点距离小于分辨率,则到达
        if(std::pow((new_point.x_ - goal_node_->pxy_.x_), 2) + std::pow((new_point.y_ - goal_node_->pxy_.y_), 2)
          <= resolution_ * resolution_)
        {
            // new_node_ptr->parent = near_node_ptr;
            // goal_node_->parent = new_node_ptr;

            //对终点chooseparent和rewire操作
            std::vector<node2d*> near_nodes;
            near_nodes = nearC(goal_node_->pxy_, resolution_ * 4);
            chooseParent(near_nodes, goal_node_);
            rewire(near_nodes, goal_node_);

            endtime = std::chrono::system_clock::now();
            std::chrono::duration<double> use_time = endtime - starttime;
            std::cout << "\033[1;32m rrtstar has use time: " << use_time.count() * 1000 << "ms. "
                      << "serach nodes number is " << nodes_.size() << "\033[0m" << std::endl;
            return true;
        }

        nodes_.push_back(new_node_ptr);
        kdtree_flann_.setInputCloud(getNodesCloudPoint().makeShared());

        endtime = std::chrono::system_clock::now();
        std::chrono::duration<double> use_time = endtime - starttime;
        if(use_time.count() > 5.0)
        {
            std::cout << "\033[1;31m rrtstar has use time more than 5s \033[0m" << std::endl;
            return false;
        }
    }
}

std::vector<node2d*> RRTStar::nearC(const vector2d &new_point, double search_radius)
{
    std::vector<int> search_indices;
    std::vector<float> search_distances;
    pcl::PointXY point;
    point.x = static_cast<float>(new_point.x_);
    point.y = static_cast<float>(new_point.y_);
    kdtree_flann_.radiusSearch(point, search_radius, search_indices, search_distances);

    std::vector<node2d*> near_nodes;
    near_nodes.reserve(search_distances.size());
    for(unsigned int i = 0; i < search_distances.size(); ++i)
    {
        near_nodes.emplace_back(nodes_[search_indices[i]]);
    }
    return near_nodes;
}

void RRTStar::chooseParent(const std::vector<node2d*> &near_nodes, node2d *const &new_node)
{
    double min_length = std::numeric_limits<double>::max();
    node2d *parent_node = nullptr;
    for(unsigned int i = 0; i < near_nodes.size(); ++i)
    {
        if(!collisionFree(near_nodes[i]->pxy_, new_node->pxy_)) continue;
        
        new_node->parent = near_nodes[i];
        double length = getPathLength(start_node_, new_node);
        if(length < min_length)
        {
            min_length = length;
            parent_node = near_nodes[i];
        }
    }
    new_node->parent = parent_node;
}

double RRTStar::getPathLength(const node2d *start_node_ptr, const node2d *goal_node_ptr)
{
    double length = 0.0;
    const node2d* temp_node_ptr = goal_node_ptr;
    while(temp_node_ptr->parent != nullptr && temp_node_ptr != start_node_ptr)
    {
        double dx = temp_node_ptr->pxy_.x_ - temp_node_ptr->parent->pxy_.x_;
        double dy = temp_node_ptr->pxy_.y_ - temp_node_ptr->parent->pxy_.y_;
        length += std::sqrt(dx * dx + dy * dy);
        temp_node_ptr = temp_node_ptr->parent;
    }
    return length;
}

void RRTStar::rewire(std::vector<node2d*> &near_nodes, const node2d *const &new_node)
{
    vector2d anode(new_node->pxy_.x_, new_node->pxy_.y_);
    for(unsigned int i = 0; i < near_nodes.size(); ++i)
    {
        vector2d nnode(near_nodes[i]->pxy_.x_, near_nodes[i]->pxy_.y_);

        if(!collisionFree(nnode, anode)) continue;

        double dist_before = getPathLength(near_nodes[i], start_node_);
        double dx = new_node->pxy_.x_ - near_nodes[i]->pxy_.x_;
        double dy = new_node->pxy_.y_ - near_nodes[i]->pxy_.y_;
        double hlen = std::sqrt(dx * dx + dy * dy);
        double dist_after = getPathLength(start_node_, new_node) + hlen;

        if(dist_after < dist_before)
        {
            near_nodes[i]->parent = const_cast<node2d*>(new_node);
        }
    }
}

}

