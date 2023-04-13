/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-15 20:18:35
 * @LastEditTime: 2023-04-05 11:01:55
 * @LastEditors: wpbit
 * @Description: 
 * @FilePath: /catkin_ws/src/rrtstar/include/rrtstar/node.hpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef NODE_HPP_
#define NODE_HPP_

#include <iostream>
#include <vector>
#include <queue>

namespace RRT
{
struct vector2d
{
    double x_;
    double y_;
    vector2d() = default;
    vector2d(const vector2d &p)
    {
        x_ = p.x_;
        y_ = p.y_;
    }
    vector2d(const double x, const double y): x_(x), y_(y) {}
};

struct vector2i
{
    int gridx_;
    int gridy_;
    vector2i() = default;
    vector2i(const vector2i &p)
    {
        gridx_ = p.gridx_;
        gridy_ = p.gridy_;
    }
    // explicit vector2i(const vector2i &p)
    // {
    //     gridx_ = p.gridx_;
    //     gridy_ = p.gridy_;
    // }
    vector2i(const int a, const int b): gridx_(a), gridy_(b) {}
};



struct node2d
{
    vector2d pxy_;
    vector2i gridxy_;
    node2d *parent;
    node2d() = default;
    explicit node2d(const vector2d &p)
    {
        pxy_ = p;
        parent = nullptr;
    }
    
};

// struct cmp
// {
//     bool operator() (const node2d &a, const node2d &b)
//     {
//         return a.f_ > b.f_;//小顶堆
//     }
// };



    
}

#endif