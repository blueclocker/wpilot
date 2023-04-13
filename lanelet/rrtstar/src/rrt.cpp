/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-15 20:19:01
 * @LastEditTime: 2023-04-06 12:58:06
 * @LastEditors: wpbit
 * @Description: 
 * @FilePath: /catkin_ws/src/rrtstar/src/rrt.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "../include/rrtstar/rrt.h"
#include <random>
#include <chrono>

namespace RRT
{

RRT::~RRT()
{
    if(map_data_ != nullptr)
    {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    // delete start_node_;
    // start_node_ = nullptr;
    delete goal_node_;
    goal_node_ = nullptr;

    // start_node_已包含在nodes_，不需另行释放
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

vector2d RRT::grid2pxy(const vector2i &p)
{
    vector2d point;
    point.x_ = ((double)p.gridx_ + 0.5) * resolution_ + pxy_lower_.x_; //origin_.x_;
    point.y_ = ((double)p.gridy_ + 0.5) * resolution_ + pxy_lower_.y_; //origin_.y_;
    return point;
}

vector2i RRT::pxy2grid(const vector2d &p)
{
    vector2i point;
    // point.gridx_ = std::min(std::max(int((p.x_ - origin_.x_) / resolution_), 0), grid_size_.gridx_ - 1);
    // point.gridy_ = std::min(std::max(int((p.y_ - origin_.y_) / resolution_), 0), grid_size_.gridy_ - 1);
    point.gridx_ = std::min(std::max(int((p.x_ - pxy_lower_.x_) / resolution_), 0), grid_size_.gridx_ - 1);
    point.gridy_ = std::min(std::max(int((p.y_ - pxy_lower_.y_) / resolution_), 0), grid_size_.gridy_ - 1);
    return point;
}

vector2d RRT::checkPoint(const vector2d &p)
{
    vector2d point;
    point.x_ = std::min(std::max(p.x_, pxy_lower_.x_), pxy_upper_.x_);
    point.y_ = std::min(std::max(p.y_, pxy_lower_.y_), pxy_upper_.y_);
    return point;
}

vector2d RRT::sample()
{
    std::random_device rd;
    std::default_random_engine eng(rd());

    std::uniform_real_distribution<> rand_pt_x = std::uniform_real_distribution<>(pxy_lower_.x_, pxy_upper_.x_);
    std::uniform_real_distribution<> rand_pt_y = std::uniform_real_distribution<>(pxy_lower_.y_, pxy_upper_.y_);

    vector2d rand_pt(rand_pt_x(eng), rand_pt_y(eng));
    return rand_pt;
}

bool RRT::isObstacle(const vector2d &p)
{
    vector2i gridxy = pxy2grid(p);
    // std::cout << gridxy.gridx_ << ", " << gridxy.gridy_ << std::endl;
    int index = all_x_grid_size_ * gridxy.gridy_ + gridxy.gridx_;
    return map_data_[index];
}

node2d* RRT::near(const vector2d &p)
{
    std::vector<int> search_indices;
    std::vector<float> search_distance;
    pcl::PointXY point;
    point.x = p.x_;
    point.y = p.y_;
    //通过kdtree找到最近的一个点
    kdtree_flann_.nearestKSearch(point, 1, search_indices, search_distance);

    if(search_indices.empty())
    {
        std::cout << "kdtree is empty" << std::endl;
    }
    return nodes_[search_indices[0]];
}

void RRT::init(const vector2d &origin_upper, const vector2d &origin_lower, const vector2i &grid_size, const double &resolution)
{
    pxy_upper_ = origin_upper;
    pxy_lower_ = origin_lower;
    grid_size_ = grid_size;
    resolution_ = resolution;

    all_x_grid_size_ = grid_size.gridx_;
    all_xy_grid_size_ = grid_size.gridx_ * grid_size.gridy_;
    map_data_ = new uint8_t[all_xy_grid_size_];
    memset(map_data_, 0, all_xy_grid_size_*sizeof(uint8_t));//?有必要
}

void RRT::setObstacle(const vector2d &p)
{
    if(p.x_ < pxy_lower_.x_ || p.x_ >= pxy_upper_.x_ ||
       p.y_ < pxy_lower_.y_ || p.y_ >= pxy_upper_.y_) return;

    vector2i gridxy = pxy2grid(p);
    int index = gridxy.gridy_ * all_x_grid_size_ + gridxy.gridx_;
    map_data_[index] = 1;
}

bool RRT::isPointValid(const vector2d &p)
{
    if(p.x_ < pxy_lower_.x_ || p.x_ >= pxy_upper_.x_ ||
       p.y_ < pxy_lower_.y_ || p.y_ >= pxy_upper_.y_) return false;

    return !isObstacle(p);
    // return true;
}

vector2d RRT::coorRounding(const vector2d &p)
{
    return grid2pxy(pxy2grid(p));
}

vector2d RRT::steer(const vector2d &rand, const vector2d &near, double step_size)
{
    vector2d res;
    double dx = rand.x_ - near.x_;
    double dy = rand.y_ - near.y_;
    double len = std::sqrt(dx * dx + dy * dy);
    dx /= len;
    dy /= len;
    res.x_ = near.x_ + dx * step_size;
    res.y_ = near.y_ + dy * step_size;
    return res;
}

pcl::PointCloud<pcl::PointXY> RRT::getNodesCloudPoint()
{
    pcl::PointCloud<pcl::PointXY> pcloud;

    for(unsigned int i = 0; i < nodes_.size(); ++i)
    {
        pcl::PointXY point;
        point.x = nodes_[i]->pxy_.x_;
        point.y = nodes_[i]->pxy_.y_;
        pcloud.push_back(point);
    }
    return pcloud;
}

bool RRT::collisionFree(const vector2d &nearp, const vector2d &newp)
{
    double step_size = resolution_ / 5.0;
    Eigen::Vector2d direction, delta, temp_point;
    Eigen::Vector2d nearpoint, newpoint;
    nearpoint << nearp.x_, nearp.y_;
    newpoint << newp.x_, newp.y_;
    direction << (newp.x_ - nearp.x_), (newp.y_ - nearp.y_);
    direction.normalized();
    delta = direction * step_size;
    temp_point << nearp.x_, nearp.y_;
    double dist_thred = (newpoint - nearpoint).norm();

    while(true)
    {
        temp_point = temp_point + delta;
        if((temp_point - nearpoint).norm() >= dist_thred)
        {
            return true;
        }

        vector2d tem(temp_point.x(), temp_point.y());
        if(isObstacle(tem))
        {
            return false;
        }
    }
}

bool RRT::search(const vector2d &start, const vector2d &goal)
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
        new_node_ptr->parent = near_node_ptr;
        nodes_.push_back(new_node_ptr);

        //采样点距离目标点距离小于分辨率,则到达
        if(std::pow((new_point.x_ - goal_node_->pxy_.x_), 2) + std::pow((new_point.y_ - goal_node_->pxy_.y_), 2)
          <= resolution_ * resolution_)
        {
            goal_node_->parent = new_node_ptr;
            endtime = std::chrono::system_clock::now();
            std::chrono::duration<double> use_time = endtime - starttime;
            std::cout << "\033[1;32m rrt has use time: " << use_time.count() * 1000 << "ms. \033[0m" << std::endl;
            return true;
        }

        kdtree_flann_.setInputCloud(getNodesCloudPoint().makeShared());

        endtime = std::chrono::system_clock::now();
        std::chrono::duration<double> use_time = endtime - starttime;
        if(use_time.count() > 5.0)
        {
            std::cout << "\033[1;31m rrt has use time more than 5s \033[0m" << std::endl;
            return false;
        }
    }
}

std::vector<vector2d> RRT::getPath()
{
    std::vector<vector2d> path;
    node2d *ptr = goal_node_;
    while(ptr != nullptr)
    {
        path.emplace_back(ptr->pxy_);
        ptr = ptr->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void RRT::reset()
{
    for(unsigned int i = 0; i < nodes_.size(); ++i)
    {
        delete nodes_[i];
        nodes_[i] = nullptr;
    }
    nodes_.clear();
}

}