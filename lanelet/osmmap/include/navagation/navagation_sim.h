/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-06 21:32:49
 * @LastEditTime: 2023-01-12 20:40:13
 * @LastEditors: wpbit
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/navagation/navagation_sim.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef NAVAGATION_SIM_H_
#define NAVAGATION_SIM_H_

#include "navagation.h"
#include <nav_msgs/Odometry.h>
#include <lgsvl_msgs/Detection3D.h>
#include <lgsvl_msgs/Detection3DArray.h>
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
    const double arrow_height_;
    const double label_height_;
    const double object_max_linear_size_ = 50.;
    double object_speed_threshold_;
    double arrow_speed_threshold_;
    double marker_display_duration_;

    int marker_id_;
    std_msgs::ColorRGBA label_color_, box_color_, hull_color_, arrow_color_, centroid_color_, model_color_;
    std::string input_topic_, ros_namespace_;
    ros::Subscriber subscriber_detected_objects_;
    ros::Publisher publisher_markers_;

    visualization_msgs::MarkerArray ObjectsToLabels(const lgsvl_msgs::Detection3DArray &in_objects);
    visualization_msgs::MarkerArray ObjectsToArrows(const lgsvl_msgs::Detection3DArray &in_objects);
    visualization_msgs::MarkerArray ObjectsToBoxes(const lgsvl_msgs::Detection3DArray &in_objects);
    visualization_msgs::MarkerArray ObjectsToModels(const lgsvl_msgs::Detection3DArray &in_objects);
    // visualization_msgs::MarkerArray ObjectsToHulls(const lgsvl_msgs::Detection3DArray &in_objects);
    visualization_msgs::MarkerArray ObjectsToCentroids(const lgsvl_msgs::Detection3DArray &in_objects);

    std::string ColorToString(const std_msgs::ColorRGBA &in_color);

    void DetectedObjectsCallback(const lgsvl_msgs::Detection3DArray &in_objects);
    bool IsObjectValid(const lgsvl_msgs::Detection3D &in_object);
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