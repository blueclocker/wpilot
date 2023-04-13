/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-16 20:39:17
 * @LastEditTime: 2022-09-16 20:50:20
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/rrtstar/include/rrtstar/rrtstar.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef RRTSTAR_H_
#define RRTSTAR_H_

#include "rrt.h"

namespace RRT
{
class RRTStar: public RRT
{
private:
    std::vector<node2d*> nearC(const vector2d &new_point, double search_radius);
    void chooseParent(const std::vector<node2d*> &near_nodes, node2d *const &new_node);
    double getPathLength(const node2d *start_node_ptr, const node2d *goal_node_ptr);
    void rewire(std::vector<node2d*> &near_nodes, const node2d *const &new_node);
public:
    RRTStar() = default;
    ~RRTStar() override;
    bool search(const vector2d &start, const vector2d &goal) override;
};








}

#endif