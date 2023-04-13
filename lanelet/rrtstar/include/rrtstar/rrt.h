/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-15 20:18:49
 * @LastEditTime: 2023-04-04 14:10:30
 * @LastEditors: wpbit
 * @Description: 
 * @FilePath: /catkin_ws/src/rrtstar/include/rrtstar/rrt.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef RRT_H_
#define RRT_H_

#include "rrtstar/node.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace RRT
{
class RRT
{
protected:
    // std::priority_queue<node2d*, std::vector<node2d*>, cmp> openlist_;
    std::vector<node2d*> nodes_;
    double resolution_;
    vector2d origin_;
    vector2d pxy_upper_;
    vector2d pxy_lower_;
    vector2i grid_size_;
    pcl::KdTreeFLANN<pcl::PointXY> kdtree_flann_;
    uint8_t *map_data_;
    int all_x_grid_size_;
    int all_xy_grid_size_;

    node2d *start_node_;
    node2d *goal_node_;
public:
    RRT() = default;
    virtual ~RRT();
    vector2d grid2pxy(const vector2i &p);
    vector2i pxy2grid(const vector2d &p);
    vector2d checkPoint(const vector2d &p);
    vector2d sample();
    bool isPointValid(const vector2d &p);
    bool isObstacle(const vector2d &p);
    node2d* near(const vector2d &p);
    void init(const vector2d &origin_upper, const vector2d &origin_lower, const vector2i &grid_size, const double &resolution);
    void setObstacle(const vector2d &p);
    vector2d coorRounding(const vector2d &p);
    vector2d steer(const vector2d &rand, const vector2d &near, double step_size);
    pcl::PointCloud<pcl::PointXY> getNodesCloudPoint();
    bool collisionFree(const vector2d &nearp, const vector2d &newp);
    virtual bool search(const vector2d &start, const vector2d &goal);
    std::vector<vector2d> getPath();
    std::vector<node2d*> getNodes() {return nodes_;}
    void reset();
};







}

#endif