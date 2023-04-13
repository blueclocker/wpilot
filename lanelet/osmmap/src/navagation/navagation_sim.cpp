/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-06 21:26:47
 * @LastEditTime: 2023-01-12 21:21:15
 * @LastEditors: wpbit
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/navagation/navagation_sim.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "navagation/navagation_sim.h"

namespace navagation
{
NavagationSim::NavagationSim(ros::NodeHandle &n) : NavagationBase(n), arrow_height_(0.5), label_height_(1.0)
{
    gps_sub_ = n_.subscribe("/odom", 1, &NavagationSim::SimCallback, this);
    //Visualize Detected Objects 

    ros_namespace_ = n.getNamespace();

    if (ros_namespace_.substr(0, 2) == "//")
    {
        ros_namespace_.erase(ros_namespace_.begin());
    }

    std::string markers_out_topic = "/objects_markers";

    std::string object_src_topic = "/boundingbox";
    ROS_INFO("[%s] objects_src_topic: %s", __APP_NAME__, object_src_topic.c_str());

    n_.param<double>("object_speed_threshold", object_speed_threshold_, 0.1);
    ROS_INFO("[%s] object_speed_threshold: %.2f", __APP_NAME__, object_speed_threshold_);

    n_.param<double>("arrow_speed_threshold", arrow_speed_threshold_, 0.25);
    ROS_INFO("[%s] arrow_speed_threshold: %.2f", __APP_NAME__, arrow_speed_threshold_);

    n_.param<double>("marker_display_duration", marker_display_duration_, 0.2);
    ROS_INFO("[%s] marker_display_duration: %.2f", __APP_NAME__, marker_display_duration_);

    std::vector<double> color;
    n_.param<std::vector<double>>("label_color", color, {255.,255.,255.,1.0});
    label_color_ = ParseColor(color);
    ROS_INFO("[%s] label_color: %s", __APP_NAME__, ColorToString(label_color_).c_str());

    n_.param<std::vector<double>>("arrow_color", color, {0.,255.,0.,0.8});
    arrow_color_ = ParseColor(color);
    ROS_INFO("[%s] arrow_color: %s", __APP_NAME__, ColorToString(arrow_color_).c_str());

    n_.param<std::vector<double>>("hull_color", color, {51.,204.,51.,0.8});
    hull_color_ = ParseColor(color);
    ROS_INFO("[%s] hull_color: %s", __APP_NAME__, ColorToString(hull_color_).c_str());

    n_.param<std::vector<double>>("box_color", color, {51.,128.,204.,0.8});
    box_color_ = ParseColor(color);
    ROS_INFO("[%s] box_color: %s", __APP_NAME__, ColorToString(box_color_).c_str());

    n_.param<std::vector<double>>("model_color", color, {190.,190.,190.,0.5});
    model_color_ = ParseColor(color);
    ROS_INFO("[%s] model_color: %s", __APP_NAME__, ColorToString(model_color_).c_str());

    n_.param<std::vector<double>>("centroid_color", color, {77.,121.,255.,0.8});
    centroid_color_ = ParseColor(color);
    ROS_INFO("[%s] centroid_color: %s", __APP_NAME__, ColorToString(centroid_color_).c_str());

    subscriber_detected_objects_ = n_.subscribe(object_src_topic, 1,
                                                &NavagationSim::DetectedObjectsCallback, this);
    ROS_INFO("[%s] object_src_topic: %s", __APP_NAME__, object_src_topic.c_str());

    publisher_markers_ = n_.advertise<visualization_msgs::MarkerArray>(markers_out_topic, 1);
    ROS_INFO("[%s] markers_out_topic: %s", __APP_NAME__, markers_out_topic.c_str());
}

void NavagationSim::SimCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //lgsvl仿真,起点相对地图原点的位姿
    Eigen::Quaterniond q_q_a = Eigen::Quaterniond(0.609886348247528, 0.0135536137968302, 0.00340890442021191, -0.792365729808807).normalized();
    Eigen::Vector3d t = Eigen::Vector3d(-8.7, 51.6, -0.7);
    
    //坐标变换
    Eigen::Vector3d v_q_a = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Vector3d localv = q_q_a.inverse().normalized() * (v_q_a - t);
    atnowpoint_->local_x_ = localv.x();
    atnowpoint_->local_y_ = localv.y();
    atnowpoint_->elevation_ = localv.z();

    //航向角变换
    start_state_[0] = atnowpoint_->local_x_;
    start_state_[1] = atnowpoint_->local_y_;
    tf::Quaternion qenu2base;
    qenu2base.setW(msg->pose.pose.orientation.w);
    qenu2base.setX(msg->pose.pose.orientation.x);
    qenu2base.setY(msg->pose.pose.orientation.y);
    qenu2base.setZ(msg->pose.pose.orientation.z);
    Eigen::Quaterniond q1 = Eigen::Quaterniond(qenu2base.w(), qenu2base.x(), qenu2base.y(), qenu2base.z()).normalized();
    Eigen::Quaterniond q = (q1 * q_q_a.inverse()).normalized();
    qenu2base.setW(q.w());
    qenu2base.setX(q.x());
    qenu2base.setY(q.y());
    qenu2base.setZ(q.z());
    start_state_[2] = tf::getYaw(qenu2base);
    // std::cout << "yaw: " << start_state_[2] << std::endl;
    
    //当前点定位到路段
    map::centerway::CenterPoint3D atnowcenterpoint = map::centerway::CenterPoint3D(*atnowpoint_);
    //当前路段精确定位到某点
    //int atnowcenterway = globalplans_->InWhichCenterway(atnowcenterpoint, nodesptr, waysptr, relationsptr);
    std::vector<int> lanelets_res = globalplans_->LocateLanelets(atnowcenterpoint, nodesptr_, waysptr_, relationsptr_);
    //ROS_INFO("lanelets number is %d!", (int)lanelets_res.size());

    //HDmap state
    osmmap::CarState hdmapstate;
    hdmapstate.header.frame_id = "map";
    hdmapstate.header.stamp = ros::Time::now();
    hdmapstate.inMap = false;
    hdmapstate.isEndExist = false;
    hdmapstate.isFindPath = false;
    hdmapstate.laneletid = 0;
    //plan
    //visualmap->pathmarkerclear();
    path_markerarray_.markers.clear();
    if(!lanelets_res.empty())
    {
        hdmapstate.inMap = true;

        //融合上一帧路径
        if(paths_.empty())
        {
            //第一帧，优先选取第一个定位结果
            start_path_ = lanelets_res[0];
            // for(int i = 0; i < lanelets_res.size(); i++)
            // {
            //     start_path_ = lanelets_res[i];
            //     if(!globalplans_->Run(start_path_, end_path_).empty())
            //     {
            //         break;
            //     }
            // }
            std::cout << "first plan!" << std::endl;
        }else{
            //其他帧，优先选取上一帧路径lanelet范围内的结果
            bool isInLastPaths = false;
            for(int i = 0; i < lanelets_res.size(); i++)
            {
                if(std::find(paths_.begin(), paths_.end(), lanelets_res[i]) != paths_.end())
                {
                    start_path_ = lanelets_res[i];
                    isInLastPaths = true;
                    std::cout << "select last frame lanelet id successed!" << std::endl;
                    break;
                }
            }
            //如果均不在上一帧lanelet范围内, 优先选取第一个
            if(!isInLastPaths)
            {
                // for(int i = 0; i < lanelets_res.size(); i++)
                // {
                //     start_path_ = lanelets_res[i];
                //     if(!globalplans_->Run(start_path_, end_path_).empty())
                //     {
                //         break;
                //     }
                // }
                start_path_ = lanelets_res[0];
            }
        }
        // start_path_ = atnowcenterway;
        
        hdmapstate.laneletid = start_path_;
        start_centerpoint_id_ = globalplans_->AtWhichPoint(atnowcenterpoint, centerwaysptr_->Find(start_path_));
        if(isend_path_exist_)
        {
            hdmapstate.isEndExist = true;
            //visualmap->pathmarkerclear();
            paths_.clear();
            paths_ = globalplans_->Run(start_path_, end_path_);
            if(!paths_.empty())
            {
                hdmapstate.isFindPath = true;
                visualmap_->Path2Marker(centerwaysptr_, paths_, path_markerarray_, currenttime_);
                PushCenterPoint(paths_);
                //smooth
                // SmoothPath();
                visualmap_->Smoothpath2Marker(smoothpathnode_, path_markerarray_, currenttime_);
                //发布导航信息
                FullNavigationInfo();
                navigation_pub_.publish(laneletinfo_);
                //发路径
                visualization_msgs::Marker golbalpath = path_markerarray_.markers.back();
                golbalpath_pub_.publish(golbalpath);
                //std::cout << "path_markerarray size is " << path_markerarray.markers.size() << std::endl;
                //发下一个车道
                FullLanesInfo(start_path_);
                lanes_pub_.publish(lanesinfo_);
            }else{
                smoothpathnode_.clear();
                smoothpathnode_.push_back(map::centerway::CenterPoint3D(*atnowpoint_));
                // double atnowpoint_heading = ConstrainAngle(msg->Heading)/180*M_PI;
                double atnowpoint_heading = start_state_[2];
                OutMapPlan(atnowcenterpoint, atnowpoint_heading);
                ROS_WARN("gps point is in reverse direction path!");
            }

            //该点到下一个路口(下一次转向)距离测试
            //double intersectiondis = centerways->length2intersection(start_centerpoint_id_, paths, relations);
            //std::cout << "distance to next intersection is " << intersectiondis << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;

        }else{
            ROS_WARN("goal point has not set!");
        } 
    }else{
        //越界：直接连接当前点与最近的道路中心点！！
        smoothpathnode_.clear();
        smoothpathnode_.push_back(map::centerway::CenterPoint3D(*atnowpoint_));
        // double atnowpoint_heading = ConstrainAngle(msg->Heading)/180*M_PI;
        double atnowpoint_heading = start_state_[2];
        
        // int nearestcenterwayid = centerwaysptr->findNearestCenterwaypointid(atnowpoint);
        // centerway::CenterPoint3D tartgetpoint;
        // centerwaysptr->nextCenterwayPointid(nearestcenterwayid, tartgetpoint);
        // smoothpathnode.push_back(tartgetpoint);

        // visualmap->smoothpath2marker(smoothpathnode, path_markerarray, currenttime);
        // //发路径
        // visualization_msgs::Marker golbalpath_ = path_markerarray.markers.back();
        // golbalpath_pub.publish(golbalpath_);
        OutMapPlan(atnowcenterpoint, atnowpoint_heading);
        
        ROS_WARN("gps point out of HD-map!");
    }

    //tf lgsvl仿真
    baselink2map_.setRotation(qenu2base);
    baselink2map_.setOrigin(tf::Vector3(atnowpoint_->local_x_, atnowpoint_->local_y_, atnowpoint_->elevation_));
    //map->rslidar->base_link
    broadcaster_.sendTransform(tf::StampedTransform(baselink2map_, ros::Time::now(), "map", "rslidar"));
    std::cout << "atnowpoint update" << std::endl;
    std::cout << "now x: " << atnowpoint_->local_x_ << ", now y: " << atnowpoint_->local_y_ << ", now z: " << atnowpoint_->elevation_ << std::endl;

    //欧拉角转四元数
    // double siny_cosp = +2.0 * (q1.w() * q1.z() + q1.x() * q1.y());
    // double cosy_cosp = +1.0 - 2.0 * (q1.y() * q1.y() + q1.z() * q1.z());
    // double yaw = atan2(siny_cosp, cosy_cosp);
    // std::cout << "head: " << (headtemp)/180*3.14159 << ", yaw: " << yaw << std::endl;
    
    //smoothpath
    // if(smoothpathnode.size() >= 2)
    // {
        // plan::Spline2D csp_obj(smoothpathnode);
        // std::vector<centerway::CenterPoint3D> temppathnode;
        // std::vector<double> rcurvature;
        // for(double i = 0; i < csp_obj.s.back(); i += 0.2)
        // {
        //     std::array<double, 3> point_ = csp_obj.calc_postion(i);
        //     centerway::CenterPoint3D pointtemp;
        //     pointtemp.x = point_[0];
        //     pointtemp.y = point_[1];
        //     pointtemp.ele = point_[2];
        //     temppathnode.push_back(pointtemp);
        //     rcurvature.push_back(csp_obj.calc_curvature(i));
        // }
    // }

    //gps path
    gpspath_.header.frame_id = "map";
    gpspath_.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = atnowpoint_->local_x_;
    pose_stamped.pose.position.y = atnowpoint_->local_y_;
    pose_stamped.pose.position.z = atnowpoint_->elevation_;
    pose_stamped.pose.orientation.x = qenu2base.x();
    pose_stamped.pose.orientation.y = qenu2base.y();
    pose_stamped.pose.orientation.z = qenu2base.z();
    pose_stamped.pose.orientation.w = qenu2base.w();
    gpspath_.poses.push_back(pose_stamped);
    gpspath_pub_.publish(gpspath_);

    //car state
    hdmapstate.heading = start_state_[2];
    hdmapstate.carPose.position.x = atnowpoint_->local_x_;
    hdmapstate.carPose.position.y = atnowpoint_->local_y_;
    hdmapstate.carPose.position.z = atnowpoint_->elevation_;
    hdmapstate.carPose.orientation.x = qenu2base.x();
    hdmapstate.carPose.orientation.y = qenu2base.y();
    hdmapstate.carPose.orientation.z = qenu2base.z();
    hdmapstate.carPose.orientation.w = qenu2base.w();
    hdmapstate.endPose = endPose_;
    hdmapstate.nextpoint.x = 0;
    hdmapstate.nextpoint.y = 0;
    hdmapstate.nextpoint.z = 0;

    //给hybid A*终点
    // if(smoothpathnode_.size() >= 2) 
    // {
    //     plan::Spline2D csp_obj(smoothpathnode_);
    //     if(csp_obj.s.back() >= 6)
    //     {
    //         std::array<double, 3> point_ = csp_obj.calc_postion(6);
    //         hdmapstate.nextpoint.x = point_[0];
    //         hdmapstate.nextpoint.y = point_[1];
    //         hdmapstate.nextpoint.z = csp_obj.calc_yaw(6);
    //     }
    // }

    //lgsvl仿真
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

    //发布速度
    std_msgs::Float32 sp;
    sp.data = hdmapstate.linear.x;
    speed_pub_.publish(std::move(sp));
}

void NavagationSim::Process()
{
    ROS_INFO("\033[1;32m --> SIM mode is working ...  \033[0m\n");
    //main loop
    ros::Rate r(10);
    while(n_.ok())
    {
        //visualmap->run();
        currenttime_ = ros::Time::now();
        map_pub_.publish(map_markerarray_);
        path_pub_.publish(path_markerarray_);
        // gridmapmsg.header.stamp = currenttime;
        // gridmap_pub.publish(gridmapmsg);
        // navigation_pub.publish(laneletinfo);
        //到达终点退出程序
        if(start_centerpoint_id_ == end_centerpoint_id_)
        {
            std::cout << "***************************************************" << std::endl;
            std::cout << std::endl;
            std::cout << "               arrvied at goal point               " << std::endl;
            std::cout << std::endl;
            std::cout << "***************************************************" << std::endl;
            break;
        }
        ros::spinOnce();
        r.sleep();
    }
}

std::string NavagationSim::ColorToString(const std_msgs::ColorRGBA &in_color)
{
    std::stringstream stream;

    stream << "{R:" << std::fixed << std::setprecision(1) << in_color.r*255 << ", ";
    stream << "G:" << std::fixed << std::setprecision(1) << in_color.g*255 << ", ";
    stream << "B:" << std::fixed << std::setprecision(1) << in_color.b*255 << ", ";
    stream << "A:" << std::fixed << std::setprecision(1) << in_color.a << "}";
    return stream.str();
}

float NavagationSim::CheckColor(double value)
{
    float final_value;
    if (value > 255.)
        final_value = 1.f;
    else if (value < 0)
        final_value = 0.f;
    else
        final_value = value/255.f;
    return final_value;
}

float NavagationSim::CheckAlpha(double value)
{
    float final_value;
    if (value > 1.)
        final_value = 1.f;
    else if (value < 0.1)
        final_value = 0.1f;
    else
        final_value = value;
    return final_value;
}

std_msgs::ColorRGBA NavagationSim::ParseColor(const std::vector<double> &in_color)
{
    std_msgs::ColorRGBA color;
    float r,g,b,a;
    if (in_color.size() == 4) //r,g,b,a
    {
        color.r = CheckColor(in_color[0]);
        color.g = CheckColor(in_color[1]);
        color.b = CheckColor(in_color[2]);
        color.a = CheckAlpha(in_color[3]);
    }
    return color;
}

void NavagationSim::DetectedObjectsCallback(const lgsvl_msgs::Detection3DArray &in_objects)
{
    visualization_msgs::MarkerArray label_markers, arrow_markers, centroid_markers, polygon_hulls, bounding_boxes,
                                    object_models;

    visualization_msgs::MarkerArray visualization_markers;

    marker_id_ = 0;

    label_markers = ObjectsToLabels(in_objects);
    arrow_markers = ObjectsToArrows(in_objects);
    // polygon_hulls = ObjectsToHulls(in_objects);
    bounding_boxes = ObjectsToBoxes(in_objects);
    object_models = ObjectsToModels(in_objects);
    centroid_markers = ObjectsToCentroids(in_objects);

    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        label_markers.markers.begin(), label_markers.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        arrow_markers.markers.begin(), arrow_markers.markers.end());
    // visualization_markers.markers.insert(visualization_markers.markers.end(),
    //                                     polygon_hulls.markers.begin(), polygon_hulls.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        bounding_boxes.markers.begin(), bounding_boxes.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        object_models.markers.begin(), object_models.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        centroid_markers.markers.begin(), centroid_markers.markers.end());

    publisher_markers_.publish(visualization_markers);

}

visualization_msgs::MarkerArray
NavagationSim::ObjectsToCentroids(const lgsvl_msgs::Detection3DArray &in_objects)
{
    visualization_msgs::MarkerArray centroid_markers;
    for (auto const &object: in_objects.detections)
    {
        if (IsObjectValid(object))
        {
            visualization_msgs::Marker centroid_marker;
            centroid_marker.lifetime = ros::Duration(marker_display_duration_);

            centroid_marker.header.stamp = in_objects.header.stamp;
            centroid_marker.header.frame_id = "rslidar";
            centroid_marker.type = visualization_msgs::Marker::SPHERE;
            centroid_marker.action = visualization_msgs::Marker::ADD;
            centroid_marker.pose = object.bbox.position;
            centroid_marker.ns = ros_namespace_ + "/centroid_markers";

            centroid_marker.scale.x = 0.25;
            centroid_marker.scale.y = 0.25;
            centroid_marker.scale.z = 0.25;

            centroid_marker.color = centroid_color_;
            centroid_marker.id = marker_id_++;
            centroid_markers.markers.push_back(centroid_marker);
        }
    }
    return centroid_markers;
}//ObjectsToCentroids

visualization_msgs::MarkerArray
NavagationSim::ObjectsToBoxes(const lgsvl_msgs::Detection3DArray &in_objects)
{
    visualization_msgs::MarkerArray object_boxes;

    for (auto const &object: in_objects.detections)
    {
        if (IsObjectValid(object) && (object.label != "unknown") 
        && (object.bbox.size.x + object.bbox.size.y + object.bbox.size.z) < object_max_linear_size_ )
        // if (IsObjectValid(object) && (object.dimensions.x + object.dimensions.y + object.dimensions.z) < object_max_linear_size_)
        {
            visualization_msgs::Marker box;

            box.lifetime = ros::Duration(marker_display_duration_);
            box.header.stamp = in_objects.header.stamp;
            box.header.frame_id = "rslidar";
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;
            box.ns = ros_namespace_ + "/box_markers";
            box.id = marker_id_++;
            box.scale = object.bbox.size;
            box.pose.position = object.bbox.position.position;
            box.pose.orientation = object.bbox.position.orientation;
            box.color = box_color_;

            object_boxes.markers.push_back(box);
        }
    }
    return object_boxes;
}//ObjectsToBoxes

visualization_msgs::MarkerArray
NavagationSim::ObjectsToModels(const lgsvl_msgs::Detection3DArray &in_objects)
{
    visualization_msgs::MarkerArray object_models;

    for (auto const &object: in_objects.detections)
    {
        if (IsObjectValid(object) && object.label != "unknown" &&
           (object.bbox.size.x + object.bbox.size.y + object.bbox.size.z) < object_max_linear_size_)
        {
            visualization_msgs::Marker model;
            model.lifetime = ros::Duration(marker_display_duration_);
            model.header.stamp = in_objects.header.stamp;
            model.header.frame_id = "rslidar";
            model.type = visualization_msgs::Marker::MESH_RESOURCE;
            model.action = visualization_msgs::Marker::ADD;
            model.ns = ros_namespace_ + "/model_markers";
            model.mesh_use_embedded_materials = false;
            model.color = model_color_;
            if(object.label == "car")
            {
                model.mesh_resource = "package://osmmap/models/car.dae";
            }
            else if (object.label == "Pedestrian")
            {
                model.mesh_resource = "package://osmmap/models/person.dae";
            }
            else if (object.label == "bicycle" || object.label == "bike")
            {
                model.mesh_resource = "package://osmmap/models/bike.dae";
            }
            else if (object.label == "bus")
            {
                model.mesh_resource = "package://osmmap/models/bus.dae";
            }
            else if(object.label == "truck")
            {
                model.mesh_resource = "package://osmmap/models/truck.dae";
            }
            else
            {
                model.mesh_resource = "package://osmmap/models/box.dae";
            }
            model.scale.x = 1;
            model.scale.y = 1;
            model.scale.z = 1;
            model.id = marker_id_++;
            model.pose.position = object.bbox.position.position;
            model.pose.position.z -= object.bbox.size.z/2;
            model.pose.orientation = object.bbox.position.orientation;

            object_models.markers.push_back(model);
        }
    }
    return object_models;
}//ObjectsToModels

// visualization_msgs::MarkerArray
// VisualizeDetectedObjects::ObjectsToHulls(const lgsvl_msgs::Detection3DArray &in_objects)
// {
//     visualization_msgs::MarkerArray polygon_hulls;

//     for (auto const &object: in_objects.detections)
//     {
//         // if (IsObjectValid(object) && !object.convex_hull.polygon.points.empty() && object.label == "unknown")
//         if (IsObjectValid(object) && !object.convex_hull.polygon.points.empty())
//         {
//             visualization_msgs::Marker hull;
//             hull.lifetime = ros::Duration(marker_display_duration_);
//             hull.header = in_objects.header;
//             hull.type = visualization_msgs::Marker::LINE_STRIP;
//             hull.action = visualization_msgs::Marker::ADD;
//             hull.ns = ros_namespace_ + "/hull_markers";
//             hull.id = marker_id_++;
//             hull.scale.x = 0.1;

//             for(auto const &point: object.convex_hull.polygon.points)
//             {
//                 geometry_msgs::Point tmp_point;
//                 tmp_point.x = point.x;
//                 tmp_point.y = point.y;
//                 tmp_point.z = point.z;
//                 hull.points.push_back(tmp_point);
//             }
//             hull.color = hull_color_;

//             polygon_hulls.markers.push_back(hull);
//         }
//     }
//     return polygon_hulls;
// }

visualization_msgs::MarkerArray
NavagationSim::ObjectsToArrows(const lgsvl_msgs::Detection3DArray &in_objects)
{
    visualization_msgs::MarkerArray arrow_markers;
    for (auto const &object: in_objects.detections)
    {
        if (IsObjectValid(object))
        {
            double velocity = object.velocity.linear.x;

            if (abs(velocity) >= arrow_speed_threshold_)
            {
                visualization_msgs::Marker arrow_marker;
                arrow_marker.lifetime = ros::Duration(marker_display_duration_);

                tf::Quaternion q(object.bbox.position.orientation.x,
                                object.bbox.position.orientation.y,
                                object.bbox.position.orientation.z,
                                object.bbox.position.orientation.w);
                double roll, pitch, yaw;

                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

                // in the case motion model fit opposite direction
                if (velocity < -0.1)
                {
                    yaw += M_PI;
                    // normalize angle
                    while (yaw > M_PI)
                        yaw -= 2. * M_PI;
                    while (yaw < -M_PI)
                        yaw += 2. * M_PI;
                }

                tf::Matrix3x3 obs_mat;
                tf::Quaternion q_tf;

                obs_mat.setEulerYPR(yaw, 0, 0);  // yaw, pitch, roll
                obs_mat.getRotation(q_tf);

                arrow_marker.header.stamp = in_objects.header.stamp;
                arrow_marker.header.frame_id = "rslidar";
                arrow_marker.ns = ros_namespace_ + "/arrow_markers";
                arrow_marker.action = visualization_msgs::Marker::ADD;
                arrow_marker.type = visualization_msgs::Marker::ARROW;
                arrow_marker.color = arrow_color_;
                arrow_marker.id = marker_id_++;

                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                arrow_marker.pose.position.x = object.bbox.position.position.x;
                arrow_marker.pose.position.y = object.bbox.position.position.y;
                arrow_marker.pose.position.z = arrow_height_;

                arrow_marker.pose.orientation.x = q_tf.getX();
                arrow_marker.pose.orientation.y = q_tf.getY();
                arrow_marker.pose.orientation.z = q_tf.getZ();
                arrow_marker.pose.orientation.w = q_tf.getW();

                // Set the scale of the arrow -- 1x1x1 here means 1m on a side
                arrow_marker.scale.x = 3;
                arrow_marker.scale.y = 0.1;
                arrow_marker.scale.z = 0.1;

                arrow_markers.markers.push_back(arrow_marker);
            }//velocity threshold
        }//valid object
    }//end for
    return arrow_markers;
}//ObjectsToArrows

visualization_msgs::MarkerArray
NavagationSim::ObjectsToLabels(const lgsvl_msgs::Detection3DArray &in_objects)
{
    visualization_msgs::MarkerArray label_markers;
    for (auto const &object: in_objects.detections)
    {
        if (IsObjectValid(object))
        {
            visualization_msgs::Marker label_marker;

            label_marker.lifetime = ros::Duration(marker_display_duration_);
            label_marker.header.stamp = in_objects.header.stamp;
            label_marker.header.frame_id = "rslidar";
            label_marker.ns = ros_namespace_ + "/label_markers";
            label_marker.action = visualization_msgs::Marker::ADD;
            label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            // label_marker.scale.x = 0.75;
            // label_marker.scale.y = 0.75;
            // label_marker.scale.z = 0.75;

            label_marker.color = label_color_;

            label_marker.id = marker_id_++;

            if(!object.label.empty() && object.label != "unknown")
                label_marker.text = object.label + " "; //Object Class if available

            std::stringstream distance_stream;
            distance_stream << std::fixed << std::setprecision(1)
                            << sqrt((object.bbox.position.position.x * object.bbox.position.position.x) +
                                        (object.bbox.position.position.y * object.bbox.position.position.y));
            std::string distance_str = distance_stream.str() + " m";
            label_marker.text += distance_str;

        // if (object.velocity_reliable)
        // {
            double velocity = object.velocity.linear.x;
            if (velocity < -0.1)
            {
                velocity *= -1;
            }

            if (abs(velocity) < object_speed_threshold_)
            {
                velocity = 0.0;
            }

            tf::Quaternion q(object.bbox.position.orientation.x, object.bbox.position.orientation.y,
                            object.bbox.position.orientation.z, object.bbox.position.orientation.w);

            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // convert m/s to km/h
            std::stringstream kmh_velocity_stream;
            kmh_velocity_stream << std::fixed << std::setprecision(1) << (velocity * 3.6);
            std::string text = "\n<" + std::to_string(object.id) + "> " + kmh_velocity_stream.str() + " km/h";
            label_marker.text += text;
        // }

            label_marker.pose.position.x = object.bbox.position.position.x;
            label_marker.pose.position.y = object.bbox.position.position.y;
            label_marker.pose.position.z = label_height_;
            label_marker.scale.x = 0.5;
            label_marker.scale.y = 0.5;
            label_marker.scale.z = 0.5;
            //label_marker.scale.z = 1.0;
            if (!label_marker.text.empty())
                label_markers.markers.push_back(label_marker);
        }
    }  // end in_objects.objects loop

    return label_markers;
}//ObjectsToLabels

bool NavagationSim::IsObjectValid(const lgsvl_msgs::Detection3D &in_object)
{
    if (//!in_object.valid ||
        std::isnan(in_object.bbox.position.orientation.x) ||
        std::isnan(in_object.bbox.position.orientation.y) ||
        std::isnan(in_object.bbox.position.orientation.z) ||
        std::isnan(in_object.bbox.position.orientation.w) ||
        std::isnan(in_object.bbox.position.position.x) ||
        std::isnan(in_object.bbox.position.position.y) ||
        std::isnan(in_object.bbox.position.position.z) ||
        (in_object.bbox.position.position.x == 0.) ||
        (in_object.bbox.position.position.y == 0.) ||
        (in_object.bbox.size.x <= 0.) ||
        (in_object.bbox.size.y <= 0.) ||
        (in_object.bbox.size.z <= 0.)
        )
    {
        return false;
    }
    return true;
}//end IsObjectValid



}//namespace navagation