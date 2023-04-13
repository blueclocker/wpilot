/*
 * @Author: wpbit
 * @Date: 2023-03-15 22:13:25
 * @LastEditors: wpbit
 * @LastEditTime: 2023-03-21 21:45:02
 * @Description: 
 */
#ifndef LATTICE_PLAN_H_
#define LATTICE_PLAN_H_

#include "lattice_trajectory.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rs_perception/PerceptionListMsg.h>
#include <osmmap/CarState.h>
#include <mutex>

namespace plan
{
class LatticePlan
{
private:
    ros::NodeHandle nh_;
    ros::Timer planner_timer_;
    geometry_msgs::Pose cur_pos_;
    geometry_msgs::Pose goal_pos_;
    double cur_speed_;
    double cur_yaw_;
    std::vector<map::centerway::CenterPoint3D> referenceline_raw_;
    FrenetPath referenceline_;
    FrenetPath last_path_;
    cpprobotics::Vec_Poi obs_list_;
    bool carstate_flag_;
    bool referenceline_flag_;
    std::mutex carstate_mutex_;
    std::mutex referenceline_mutex_;
    std::mutex obs_mutex_;

    double c_speed_ = 10.0 / 3.6;
    double c_d_ = 2.0;
    double c_dd_ = 0.0;
    double c_ddd_ = 0.0;
    double s0_ = 0.0;

    ros::Subscriber obs_sub_;
    ros::Subscriber car_sub_;
    ros::Subscriber globalpath_sub_;
    ros::Publisher frenetpath_pub_;

    double DistanceXY(double a, double b, double c, double d);
public:
    LatticePlan(ros::NodeHandle &n);
    ~LatticePlan();
    void CarstateCallback(const osmmap::CarState::ConstPtr &msg);
    void GolbalpathCallback(const visualization_msgs::Marker::ConstPtr &msg);
    void ObsCallback(const rs_perception::PerceptionListMsg::ConstPtr &msg);
    void PlannerLoop(const ros::TimerEvent &);
    int GetNearestReferencelineIndex(double x, double y);
    double GetNearestReferencelineLength(double x, double y);
    double GetNearestReferencelineLatDist(double x, double y);
    bool LeftofLine(const double &p1x, const double &p1y, const double &p2x, const double &p2y);
};


} // namespace plan

#endif