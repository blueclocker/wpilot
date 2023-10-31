/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-06 21:32:49
 * @LastEditTime: 2023-10-31 16:06:09
 * @LastEditors: wpbit
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/navagation/navagation_sim.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef NAVAGATION_SIM_H_
#define NAVAGATION_SIM_H_

#include "navagation.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define __APP_NAME__ "visualize_detected_objects"

namespace navagation
{
class NavagationSim : public NavagationBase
{
private:
    void SimCallback(const nav_msgs::Odometry::ConstPtr &msg);
    std_msgs::ColorRGBA label_color_, box_color_, hull_color_, arrow_color_, centroid_color_, model_color_;

    std::string ColorToString(const std_msgs::ColorRGBA &in_color);

    float CheckColor(double value);
    float CheckAlpha(double value);
    std_msgs::ColorRGBA ParseColor(const std::vector<double> &in_color);
public:
    NavagationSim(ros::NodeHandle &n);
    virtual ~NavagationSim() override = default;
    virtual void Process() override;
};

}//namespace navagation

#endif