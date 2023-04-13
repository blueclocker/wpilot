/*
 * @Author: wpbit
 * @Date: 2023-04-04 09:13:00
 * @LastEditors: wpbit
 * @LastEditTime: 2023-04-07 16:28:04
 * @Description: 
 */
#include "rrtstar/navagation_rrtstar.h"
#include <tf/tf.h>

namespace RRT
{
NavagationRRTstar::NavagationRRTstar(ros::NodeHandle &nh) : nh_(nh)
{
    nh_.getParam("map_path", map_path_);
    nh_.getParam("resolution", resolution_);
    nh_.getParam("origin_x", origin_x_);
    nh_.getParam("origin_y", origin_y_);
    nh_.getParam("origin_z", origin_z_);
    nh_.getParam("isobs", isobs_);
    map_valid_ = false;
    start_valid_ = false;
    //test
    start_valid_ = true;
    start_point_.x_ = 0;
    start_point_.y_ = 0;
    //test
    goal_valid_ = false;
    rrt_plan_ = new RRTStar();
    readMap(map_path_);
    goal_point_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &NavagationRRTstar::goalPointCallBack, this);
    localization_sub_ = nh_.subscribe("/mapping_odometry", 1, &NavagationRRTstar::localizationCallBack, this);
    path_pub_ = nh_.advertise<visualization_msgs::Marker>("/navagation_node/golbalpath_info", 1);
    carstate_pub_ = nh_.advertise<osmmap::CarState>("/navagation_node/carstate_info", 1);
    gpspath_pub_ = nh_.advertise<nav_msgs::Path>("/navagation_node/gpspath_info", 1);
    nodes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("rrtnodes", 1);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/navagation_node/OccupancyGridMap", 1);
}

NavagationRRTstar::~NavagationRRTstar()
{
    delete rrt_plan_;
}

void NavagationRRTstar::run()
{
    ros::Rate r(10);
    while(nh_.ok())
    {
        map_.header.stamp = ros::Time::now();
        map_pub_.publish(map_);
        visualizationPath();
        ros::spinOnce();
        r.sleep();
    }
}

void NavagationRRTstar::goalPointCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_point_.x_ = msg->pose.position.x;
    goal_point_.y_ = msg->pose.position.y;
    endPose_ = msg->pose;
    if(rrt_plan_->isPointValid(goal_point_)) 
    {
        goal_valid_ = true;
        ROS_INFO("get goal point x: %f, y: %f", goal_point_.x_, goal_point_.y_);

        if(!start_valid_ || !map_valid_) 
        {
            ROS_WARN("start point invalid or map invalid!");
            return;
        }

        ROS_INFO("rrtstar searching");
        if(rrt_plan_->search(start_point_, goal_point_))
        {
            path_ = rrt_plan_->getPath();
            // auto nodes = rrt_plan_->getNodes();
            // visualizationNodes(nodes);
            rrt_plan_->reset();
            ROS_INFO("rrtstar find!");
        }else{
            ROS_INFO("rrtstar failed!");
        }
    }else{
        ROS_WARN("invalid goal point!");
    }
}

void NavagationRRTstar::localizationCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    start_point_.x_ = msg->pose.pose.position.x;
    start_point_.y_ = msg->pose.pose.position.y;
    if(rrt_plan_->isPointValid(start_point_)) start_valid_ = true;
    ROS_INFO("get start point x: %f, y: %f", start_point_.x_, start_point_.y_);

    tf::Quaternion qenu2base;
    qenu2base.setW(msg->pose.pose.orientation.w);
    qenu2base.setX(msg->pose.pose.orientation.x);
    qenu2base.setY(msg->pose.pose.orientation.y);
    qenu2base.setZ(msg->pose.pose.orientation.z);

    //carstate
    osmmap::CarState hdmapstate;
    hdmapstate.header.frame_id = "map";
    hdmapstate.header.stamp = ros::Time::now();
    hdmapstate.inMap = false;
    hdmapstate.isEndExist = false;
    hdmapstate.isFindPath = false;
    hdmapstate.laneletid = 0;
    hdmapstate.heading = tf::getYaw(qenu2base);
    hdmapstate.carPose.position.x = start_point_.x_;
    hdmapstate.carPose.position.y = start_point_.y_;
    hdmapstate.carPose.position.z = origin_z_;
    hdmapstate.carPose.orientation.x = qenu2base.x();
    hdmapstate.carPose.orientation.y = qenu2base.y();
    hdmapstate.carPose.orientation.z = qenu2base.z();
    hdmapstate.carPose.orientation.w = qenu2base.w();
    hdmapstate.endPose = endPose_;
    hdmapstate.nextpoint.x = 0;
    hdmapstate.nextpoint.y = 0;
    hdmapstate.nextpoint.z = 0;
    //角速度、线速度
    hdmapstate.linear.x = msg->twist.twist.linear.x;
    hdmapstate.linear.y = msg->twist.twist.linear.y;
    hdmapstate.linear.z = msg->twist.twist.linear.z;
    hdmapstate.angular.x = msg->twist.twist.angular.x;
    hdmapstate.angular.y = msg->twist.twist.angular.y;
    hdmapstate.angular.z = msg->twist.twist.angular.z;
    //暂时加速度均给0，规划未用到
    hdmapstate.Accell.x = 0;
    hdmapstate.Accell.y = 0;
    hdmapstate.Accell.z = 0;
    carstate_pub_.publish(hdmapstate);

    //gps path
    gpspath_.header.frame_id = "map";
    gpspath_.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = start_point_.x_;
    pose_stamped.pose.position.y = start_point_.y_;
    pose_stamped.pose.position.z = origin_z_;
    pose_stamped.pose.orientation.x = qenu2base.x();
    pose_stamped.pose.orientation.y = qenu2base.y();
    pose_stamped.pose.orientation.z = qenu2base.z();
    pose_stamped.pose.orientation.w = qenu2base.w();
    gpspath_.poses.push_back(pose_stamped);
    gpspath_pub_.publish(gpspath_);
}

void NavagationRRTstar::readMap(const std::string &map_path)
{
    cv::Mat image = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
    int rows = image.rows;
    int cols = image.cols;
    std::cout << "rows: " << rows << ", cols: " << cols << std::endl;
    if(image.data == nullptr) 
    {
        ROS_WARN("read map failed!");
        return;
    }
    map_.header.frame_id = "map";
    map_.info.origin.position.x = origin_x_;
    map_.info.origin.position.y = origin_y_;
    map_.info.origin.position.z = origin_z_;
    map_.info.origin.orientation.w = 1.0;
    map_.info.resolution = resolution_;
    map_.info.width = rows;
    map_.info.height = cols;
    map_.data.resize(rows * cols);
    
    vector2d originxylow, originxyup;
    vector2i gridsize;
    originxylow.x_ = origin_x_;
    originxylow.y_ = origin_y_;
    originxyup.x_ = originxylow.x_ + resolution_ * rows;
    originxyup.y_ = originxylow.y_ + resolution_ * cols;
    gridsize.gridx_ = rows;
    gridsize.gridy_ = cols;
    rrt_plan_->init(originxyup, originxylow, gridsize, resolution_);
    // ROS_INFO("map ld point x: %f, y: %f", originxylow.x_, originxylow.y_);
    // ROS_INFO("map ru point x: %f, y: %f", originxyup.x_, originxyup.y_);
    ROS_INFO("map init successful!");
    
//     索引号沿着x轴方向逐行累加,取值如下图所示:
//                                ^
//     ---------------------------| x
//     |   |        ...       |w-1|  
//     |---|----------------------|
//     |            ...           |  
//     |---|--------------|---|---|
//     |   |        ...   |w+1| 1 |
//     |---|--------------|---|---|
//     |   |        ...   | w | 0 |
//  <-----------------------------|---
//     y                     OccupacyGrid_origin

    for(int x = 0; x < rows; ++x)
    {
        unsigned char *data = image.ptr<uchar>(rows - 1 - x);
        for(int y = 0; y < cols; ++y)
        {
            // map_.data[y * rows + x] = 100 - *(data + cols - 1 - y) * 100 / 255;
            int t = 100 - *(data + cols - 1 - y) * 100 / 255;
            // if(map_.data[y * rows + x] >= isobs_)
            if(t <= isobs_)
            {
                vector2d obsp;
                obsp.x_ = x * resolution_ + origin_x_;
                obsp.y_ = y * resolution_ + origin_y_;
                rrt_plan_->setObstacle(obsp);
                map_.data[y * rows + x] = 100;
            }
        }
    }
    map_valid_ = true;
    //cv::imshow("map", image);
}

void NavagationRRTstar::visualizationPath()
{
    if(path_.size() < 2) return;
    visualization_msgs::Marker maker;
    maker.header.frame_id = "map";
    maker.header.stamp = ros::Time::now();
    maker.type = visualization_msgs::Marker::LINE_STRIP;
    maker.action = visualization_msgs::Marker::ADD;
    maker.ns = "rrtpath";
    maker.color.a = 1.0;
    maker.color.g = 1.0;
    maker.pose.orientation.w = 1.0;
    maker.scale.x = 0.25;
    maker.id = 1;
    for(int i = 0; i < path_.size(); ++i)
    {
        geometry_msgs::Point pos;
        pos.x = path_[i].x_;
        pos.y = path_[i].y_;
        pos.z = origin_z_;
        maker.points.push_back(pos);
    }
    path_pub_.publish(maker);
}

void NavagationRRTstar::visualizationNodes(const std::vector<node2d*> &nodes)
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





}//namespace rrt

