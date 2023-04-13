/*
 * @Author: wpbit
 * @Date: 2023-03-15 22:13:35
 * @LastEditors: wpbit
 * @LastEditTime: 2023-03-21 21:44:34
 * @Description: 
 */
#include "plan/lattice_plan.h"
#include <tf/tf.h>


namespace plan
{
LatticePlan::LatticePlan(ros::NodeHandle &n) : nh_(n)
{
    carstate_flag_ = false;
    referenceline_flag_ = false;
    frenetpath_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("frenet_path", 1);
    obs_sub_ = nh_.subscribe("/adaptive_clustering/rs_percept_result", 1, &LatticePlan::ObsCallback, this);
    car_sub_ = nh_.subscribe("/navagation_node/carstate_info", 1, &LatticePlan::CarstateCallback, this);
    globalpath_sub_ = nh_.subscribe("/navagation_node/golbalpath_info", 1, &LatticePlan::GolbalpathCallback, this);
    planner_timer_ = nh_.createTimer(ros::Duration(0.1), &LatticePlan::PlannerLoop, this);
    ros::spin();
}

LatticePlan::~LatticePlan()
{
}

double LatticePlan::DistanceXY(double a, double b, double c, double d)
{
    return std::sqrt((a-b)*(a-b) + (c-d)*(c-d));
}

void LatticePlan::CarstateCallback(const osmmap::CarState::ConstPtr &msg)
{
    // std::cout << "get carstate" << std::endl;
    carstate_mutex_.lock();
    cur_pos_ = msg->carPose;
    goal_pos_ = msg->endPose;
    cur_speed_ = msg->linear.x;
    // tf::Quaternion q;
    // tf::quaternionMsgToTF(msg->carPose.orientation, q);
    cur_yaw_ = tf::getYaw(msg->carPose.orientation);
    carstate_mutex_.unlock();
    carstate_flag_ = true;
    // std::cout << "get car at x: " << cur_pos_.position.x << ", y: " << cur_pos_.position.y << std::endl;
}

void LatticePlan::GolbalpathCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
    referenceline_mutex_.lock();
    referenceline_raw_.clear();
    map::centerway::CenterPoint3D point;
    for(int i = 0; i < msg->points.size(); ++i)
    {
        point.x_ = msg->points[i].x;
        point.y_ = msg->points[i].y;
        point.ele_ = msg->points[i].z;
        referenceline_raw_.push_back(point);
    }
    
    plan::Spline2D csp_obj(referenceline_raw_);
    referenceline_.clear();
    for(double i = 0.0; i <= csp_obj.s.back(); i += 0.2)
    {
        std::array<double, 3> p = csp_obj.calc_postion(i);
        referenceline_.x.push_back(p[0]);
        referenceline_.y.push_back(p[1]);
        referenceline_.yaw.push_back(csp_obj.calc_yaw(i));
        referenceline_.c.push_back(csp_obj.calc_curvature(i));
        referenceline_.ds.push_back(i);
    }
    referenceline_mutex_.unlock();
    referenceline_flag_ = true;

    //visualization
    // visualization_msgs::Marker frenet_path_marker;
    // frenet_path_marker.header.frame_id = "map";
    // frenet_path_marker.header.stamp = ros::Time::now();
    // frenet_path_marker.action = visualization_msgs::Marker::ADD;
    // frenet_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    // frenet_path_marker.ns = "referenceline_path";
    // frenet_path_marker.id = 1;
    // frenet_path_marker.color.a = 1.0;
    // frenet_path_marker.color.b = 1.0;
    // frenet_path_marker.scale.x = 0.2;
    // frenet_path_marker.pose.orientation.w = 1.0;
    // for(int i = 0; i < referenceline_.x.size(); ++i)
    // {
    //     geometry_msgs::Point p;
    //     p.x = referenceline_.x[i];
    //     p.y = referenceline_.y[i];
    //     frenet_path_marker.points.push_back(p);
    // }
    // frenetpath_pub_.publish(frenet_path_marker);
    // std::cout << "reference line size is: " << referenceline_.x.size() << std::endl;
}

void LatticePlan::ObsCallback(const rs_perception::PerceptionListMsg::ConstPtr &msg)
{
    std::lock(obs_mutex_, carstate_mutex_);
    obs_list_.clear();
    for(int i = 0; i < msg->perceptions.size(); ++i)
    {
        if(msg->perceptions[i].centroid_y > 4.0) continue;
        for(int j = 0; j < msg->perceptions[i].polygon_point.size(); ++j)
        {
            cpprobotics::Poi_f point;
            double xv = msg->perceptions[i].polygon_point[j].x;
            double yv = msg->perceptions[i].polygon_point[j].y;
            double phiv = cur_yaw_;
            double xo = xv * cos(phiv) - yv * sin(phiv) + cur_pos_.position.x;
            double yo = xv * sin(phiv) + yv * cos(phiv) + cur_pos_.position.y;
            point[0] = xo;
            point[1] = yo;
            obs_list_.push_back(point);
        }
    }
    obs_mutex_.unlock();
    carstate_mutex_.unlock();
    std::cout << "get obs size is: " << obs_list_.size() << std::endl;
}

void LatticePlan::PlannerLoop(const ros::TimerEvent &)
{
    if(!carstate_flag_ || !referenceline_flag_) return;
    const auto start_timestamp = std::chrono::system_clock::now();
    std::lock(carstate_mutex_, referenceline_mutex_, obs_mutex_);
    double ego_s = GetNearestReferencelineLength(cur_pos_.position.x, cur_pos_.position.y);
    double ego_l = GetNearestReferencelineLatDist(cur_pos_.position.x, cur_pos_.position.y);
    // ROS_INFO("ego_s: %f, ego_l: %f", ego_s, ego_l);
    s0_ = ego_s;
    plan::Spline2D csp_obj(referenceline_raw_);
    double theta_r = csp_obj.calc_yaw(ego_s);
    double kappa_r = csp_obj.calc_curvature(ego_s);
    if(std::abs(cur_speed_) > 1e-3)
    {
        c_speed_ = cur_speed_ * std::cos(cur_yaw_ - theta_r) / (1.0 - kappa_r * ego_l);
    }
    c_d_ = ego_l;
    // c_dd_ = (1.0 - kappa_r * ego_l) * std::tan(cur_yaw_ - theta_r);

    FrenetOptimalTrajectory frenet_optimal_trajectory;
    FrenetPath final_path = frenet_optimal_trajectory.FrenetOptimalPlanning(
        csp_obj, s0_, c_speed_, c_d_, c_dd_, c_ddd_, obs_list_ 
    );
    std::cout << "state: " << final_path.s.empty() << std::endl;
    if(!final_path.s.empty())
    {
        std::cout << "plan successful!" << std::endl;

        //visualization
        visualization_msgs::Marker frenet_path_marker;
        frenet_path_marker.header.frame_id = "map";
        frenet_path_marker.header.stamp = ros::Time::now();
        frenet_path_marker.action = visualization_msgs::Marker::ADD;
        frenet_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        frenet_path_marker.ns = "frenet_path";
        frenet_path_marker.id = 0;
        frenet_path_marker.color.a = 1.0;
        frenet_path_marker.color.b = 1.0;
        frenet_path_marker.scale.x = 0.2;
        frenet_path_marker.pose.orientation.w = 1.0;
        for(int i = 0; i < final_path.x.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = final_path.x[i];
            p.y = final_path.y[i];
            frenet_path_marker.points.push_back(p);
        }

        visualization_msgs::Marker samplepoints_marker;
        samplepoints_marker.header.frame_id = "map";
        samplepoints_marker.header.stamp = ros::Time::now();
        samplepoints_marker.action = visualization_msgs::Marker::ADD;
        samplepoints_marker.type = visualization_msgs::Marker::POINTS;
        samplepoints_marker.ns = "sample_points";
        samplepoints_marker.id = 1;
        samplepoints_marker.color.a = 0.5;
        samplepoints_marker.color.b = 1.0;
        samplepoints_marker.scale.x = 0.1;
        samplepoints_marker.scale.y = 0.1;
        samplepoints_marker.pose.orientation.w = 1.0;
        frenet_optimal_trajectory.VisualizationSamplePoint(csp_obj, s0_, c_speed_, c_d_, c_dd_, c_ddd_, samplepoints_marker);

        visualization_msgs::MarkerArray vis_marker;
        vis_marker.markers.push_back(frenet_path_marker);
        vis_marker.markers.push_back(samplepoints_marker);
        frenetpath_pub_.publish(vis_marker);

        //????
        s0_ = final_path.s[1];
        c_d_ = final_path.d[1];
        c_dd_ = final_path.d_d[1];
        c_ddd_ = final_path.d_dd[1];
        c_speed_ = final_path.s_d[1];
        last_path_ = final_path;
    }else{
        std::cout << "plan failed, use last path!" << std::endl;

        //visualization
        visualization_msgs::Marker frenet_path_marker;
        frenet_path_marker.header.frame_id = "map";
        frenet_path_marker.header.stamp = ros::Time::now();
        frenet_path_marker.action = visualization_msgs::Marker::ADD;
        frenet_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        frenet_path_marker.ns = "frenet_path";
        frenet_path_marker.id = 0;
        frenet_path_marker.color.a = 1.0;
        frenet_path_marker.color.b = 1.0;
        frenet_path_marker.scale.x = 0.2;
        frenet_path_marker.pose.orientation.w = 1.0;
        for(int i = 0; i < last_path_.x.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = last_path_.x[i];
            p.y = last_path_.y[i];
            frenet_path_marker.points.push_back(p);
        }

        visualization_msgs::Marker samplepoints_marker;
        samplepoints_marker.header.frame_id = "map";
        samplepoints_marker.header.stamp = ros::Time::now();
        samplepoints_marker.action = visualization_msgs::Marker::ADD;
        samplepoints_marker.type = visualization_msgs::Marker::POINTS;
        samplepoints_marker.ns = "sample_points";
        samplepoints_marker.id = 1;
        samplepoints_marker.color.a = 0.5;
        samplepoints_marker.color.b = 1.0;
        samplepoints_marker.scale.x = 0.1;
        samplepoints_marker.scale.y = 0.1;
        samplepoints_marker.pose.orientation.w = 1.0;
        frenet_optimal_trajectory.VisualizationSamplePoint(csp_obj, s0_, c_speed_, c_d_, c_dd_, c_ddd_, samplepoints_marker);

        visualization_msgs::MarkerArray vis_marker;
        vis_marker.markers.push_back(frenet_path_marker);
        vis_marker.markers.push_back(samplepoints_marker);
        frenetpath_pub_.publish(vis_marker);

        c_dd_ = 0.0;
        c_ddd_ = 0.0;
        c_speed_ = 10.0 / 3.6;
    }

    const auto end_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_timestamp - start_timestamp;
    std::cout << "plan loop total time is " << diff.count() * 1000.0 << " ms." << std::endl;

    carstate_mutex_.unlock();
    referenceline_mutex_.unlock();
    obs_mutex_.unlock();
}

int LatticePlan::GetNearestReferencelineIndex(double x, double y)
{
    // auto distance = [](double a, double b, double c, double d)
    // {
    //     return std::sqrt(std::pow((a-b), 2) + std::pow((c-d), 2));
    // };
    
    double min_distance = std::numeric_limits<double>::max();
    int index = 0;
    for(int i = 0; i < referenceline_.x.size(); ++i)
    {
        double tmpe = DistanceXY(x, referenceline_.x[i], y, referenceline_.y[i]);
        if(tmpe <= min_distance)
        {
            min_distance = tmpe;
            index = i;
        }
    }
    // std::cout << "nearest reference distance: " << min_distance << ", index: " << index << std::endl;
    return index;
}

double LatticePlan::GetNearestReferencelineLength(double x, double y)
{
    // auto distance = [](double a, double b, double c, double d)
    // {
    //     return std::sqrt(std::pow((a-b), 2) + std::pow((c-d), 2));
    // };
    // double sum = 0.0;
    // for(int i = 1; i <= GetNearestReferencelineIndex(); ++i)
    // {
    //     sum += distance(referenceline_.x[i], referenceline_.x[i-1],
    //                     referenceline_.y[i], referenceline_.y[i-1]);
    // }
    // return sum;
    return referenceline_.ds[GetNearestReferencelineIndex(x, y)];
}

double LatticePlan::GetNearestReferencelineLatDist(double x, double y)
{
    // auto distance = [](double a, double b, double c, double d)
    // {
    //     return std::sqrt(std::pow((a-b), 2) + std::pow((c-d), 2));
    // };
    double min_distance = std::numeric_limits<double>::max();
    int index = 0;
    for(int i = 0; i < referenceline_.x.size() - 1; ++i)
    {
        double tmp = DistanceXY(x, referenceline_.x[i], y, referenceline_.y[i]);
        if(tmp <= min_distance)
        {
            min_distance = tmp;
            index = i;
        }
    }
    int sign = LeftofLine(referenceline_.x[index], referenceline_.y[index], 
                          referenceline_.x[index + 1], referenceline_.y[index + 1]) ? 1 : -1;
    return sign * min_distance;
}

bool LatticePlan::LeftofLine(const double &p1x, const double &p1y, const double &p2x, const double &p2y)
{
    // double tmpx = (p1x - p2x) / (p1y - p2y) * (cur_pos_.position.y - p2y) + p2x;
    // if(tmpx > cur_pos_.position.x) return true;
    // else return false;

    //a向量是p1p2向量逆时针转90度，为标准正方向
    double ax = -p2y + p1y;
    double ay = p2x - p1x;
    //pp1向量
    double bx = p1x - cur_pos_.position.x;
    double by = p1y - cur_pos_.position.y;
    if(ax*bx + ay*by < 0) return true;
    else return false;
}


}//namespace plan

