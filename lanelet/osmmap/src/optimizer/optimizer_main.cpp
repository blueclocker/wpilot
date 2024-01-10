#include "optimizer/optimizer.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

// 变量
ros::Publisher obs_pub;
ros::Publisher path_pub;
double sx;
double sy;
double sphi;
double sv;
double ex;
double ey;
double ephi;
bool has_start;
bool has_end;
optimizer::PlannerResult result;
optimizer::PlannerOpenSpaceConfig planner_open_space_config_;
std::unique_ptr<optimizer::UnionPlanner> hybrid_test_ = 
std::unique_ptr<optimizer::UnionPlanner>(new optimizer::UnionPlanner(planner_open_space_config_));
std::vector<double> XYbounds{0.0, 80.0, 0.0, 50.0};
std::vector<std::vector<optimizer::Vec2d>> obstacles_list;


void visualizationObstacles(const std::vector<std::vector<optimizer::Vec2d>> &obstacles_lists)
{
    visualization_msgs::MarkerArray obses;
    visualization_msgs::Marker obs;
    obs.header.frame_id = "map";
    obs.header.stamp = ros::Time::now();
    obs.action = visualization_msgs::Marker::ADD;
    obs.type = visualization_msgs::Marker::LINE_STRIP;
    obs.ns = "obstacle";
    // obs.lifetime = ros::Duration(0.1);
    obs.pose.orientation.w = 1.0;
    obs.color.a = 1.0;
    obs.color.r = 1.0;
    obs.scale.x = 0.1;
    int index = 0;
    // obs.id = 1;
    for(auto p : obstacles_lists)
    {
        obs.points.clear();
        obs.id = index++;
        for(auto pp : p)
        {
            geometry_msgs::Point a;
            a.x = pp.x();
            a.y = pp.y();
            a.z = 0;
            obs.points.push_back(a);
        }
        obses.markers.push_back(obs);
    }
    obs_pub.publish(obses);
}

void visualizationPath(optimizer::PlannerResult &path)
{
    if(path.x.size() == 0) return;
    visualization_msgs::MarkerArray paths;

    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    // m.lifetime = ros::Duration(0.1);
    m.ns = "path";
    m.id = 1;
    m.pose.orientation.w = 1.0;
    m.color.a = 1.0;
    m.color.b = 1.0;
    m.scale.x = 0.1;
    CHECK_EQ(path.x.size(), path.y.size());
    // CHECK_EQ(path.x.size(), path.phi.size());

    visualization_msgs::Marker vehicle;
    vehicle.header.frame_id = "map";
    vehicle.header.stamp = ros::Time::now();
    vehicle.type = visualization_msgs::Marker::CUBE;
    // vehicle.id = static_cast<int>(i / vehicle_interval);
    vehicle.ns = "vehicle";
    vehicle.scale.x = 4.933;//2.11
    vehicle.scale.y = 2.11;//4.933
    vehicle.scale.z = 0.01;
    vehicle.color.a = 0.1;
    vehicle.color.r = 1.0;
    vehicle.color.b = 1.0;
    vehicle.color.g = 0.0;

    visualization_msgs::Marker path_point;
    path_point.header.frame_id = "map";
    path_point.header.stamp = ros::Time::now();
    path_point.type = visualization_msgs::Marker::SPHERE_LIST;
    path_point.action = visualization_msgs::Marker::ADD;
    // vehicle.id = static_cast<int>(i / vehicle_interval);
    path_point.ns = "path_points";
    path_point.scale.x = 0.3;
    path_point.id = 1;
    path_point.pose.orientation.w = 1.0;
    path_point.color.r = 1.0;
    path_point.color.a = 1.0;

    for(int i = 0; i < path.x.size(); ++i)
    {
        geometry_msgs::Point a;
        a.x = path.x[i];
        a.y = path.y[i];
        a.z = 0;
        m.points.push_back(a);
        path_point.points.push_back(a);

        vehicle.id = i;
        vehicle.pose.position.x = a.x;
        vehicle.pose.position.y = a.y;
        vehicle.pose.position.z = 0.0;
        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path.phi[i]);
        paths.markers.push_back(vehicle);
    }

    paths.markers.push_back(m);
    paths.markers.push_back(path_point);
    path_pub.publish(paths);
}

void StartpointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    sx = msg->pose.pose.position.x;
    sy = msg->pose.pose.position.y;
    sphi = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("get start point: x = %f, y = %f", sx, sy);
    has_start = true;

    if(has_end && has_start) 
    {
        ROS_INFO("start plan ...");
        result.clear();
        bool find = hybrid_test_->Plan(sx, sy, sphi, 0.0, ex, ey, ephi, XYbounds, obstacles_list, &result);
        if(!find) return;
        std::cout << "the number of find path is " << result.x.size() << std::endl;
        visualizationPath(result);
        // static tf::TransformBroadcaster transform_broadcaster;
        // for (int i = 0; i < result_.x.size(); ++i) 
        // {
        //     tf::Transform transform;
        //     transform.setOrigin(tf::Vector3(result_.x[i], result_.y[i], 0.0));
        //     tf::Quaternion q;
        //     q.setRPY(0, 0, result_.phi[i]);
        //     transform.setRotation(q);
        //     transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ground_link"));

        //     ros::Duration(0.05).sleep();
        // }
    }
}

void GoalpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ex = msg->pose.position.x;
    ey = msg->pose.position.y;
    ephi = tf::getYaw(msg->pose.orientation);
    ROS_INFO("get goal point: x = %f, y = %f", ex, ey);
    has_end = true;

    if(has_end && has_start)
    {
        ROS_INFO("start plan ...");
        result.clear();
        bool find = hybrid_test_->Plan(sx, sy, sphi, 0.0, ex, ey, ephi, XYbounds, obstacles_list, &result);
        if(!find) return;
        std::cout << "the number of find path is " << result.x.size() << std::endl;
        visualizationPath(result);
        // static tf::TransformBroadcaster transform_broadcaster;
        // for (int i = 0; i < result_.x.size(); ++i) 
        // {
        //     tf::Transform transform;
        //     transform.setOrigin(tf::Vector3(result_.x[i], result_.y[i], 0.0));
        //     tf::Quaternion q;
        //     q.setRPY(0, 0, result_.phi[i]);
        //     transform.setRotation(q);
        //     transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ground_link"));

        //     ros::Duration(0.05).sleep();
        // }
    }
}

void setObs()
{
    ROS_INFO("set obstacles!");
    obstacles_list.clear();
    std::vector<optimizer::Vec2d> a_obstacle;
    a_obstacle.emplace_back(10.0, 13.0);
    a_obstacle.emplace_back(12.0, 13.0);
    a_obstacle.emplace_back(12.0, 18.0);
    a_obstacle.emplace_back(10.0, 18.0);
    a_obstacle.emplace_back(10.0, 13.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(16.0, 13.0);
    a_obstacle.emplace_back(18.0, 13.0);
    a_obstacle.emplace_back(18.0, 17.0);
    a_obstacle.emplace_back(16.0, 17.0);
    a_obstacle.emplace_back(16.0, 13.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(19.0, 12.0);
    a_obstacle.emplace_back(21.0, 12.0);
    a_obstacle.emplace_back(21.0, 17.0);
    a_obstacle.emplace_back(19.0, 17.0);
    a_obstacle.emplace_back(19.0, 12.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(22.0, 11.0);
    a_obstacle.emplace_back(24.0, 11.0);
    a_obstacle.emplace_back(24.0, 15.0);
    a_obstacle.emplace_back(22.0, 15.0);
    a_obstacle.emplace_back(22.0, 11.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(28.0, 12.0);
    a_obstacle.emplace_back(30.0, 12.0);
    a_obstacle.emplace_back(30.0, 17.0);
    a_obstacle.emplace_back(28.0, 17.0);
    a_obstacle.emplace_back(28.0, 12.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(33.0, 13.0);
    a_obstacle.emplace_back(35.0, 13.0);
    a_obstacle.emplace_back(35.0, 18.0);
    a_obstacle.emplace_back(33.0, 18.0);
    a_obstacle.emplace_back(33.0, 13.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(36.0, 12.0);
    a_obstacle.emplace_back(38.0, 12.0);
    a_obstacle.emplace_back(38.0, 16.0);
    a_obstacle.emplace_back(36.0, 16.0);
    a_obstacle.emplace_back(36.0, 12.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(39.0, 13.0);
    a_obstacle.emplace_back(41.0, 13.0);
    a_obstacle.emplace_back(41.0, 17.0);
    a_obstacle.emplace_back(39.0, 17.0);
    a_obstacle.emplace_back(39.0, 13.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(42.0, 12.0);
    a_obstacle.emplace_back(44.0, 12.0);
    a_obstacle.emplace_back(44.0, 17.0);
    a_obstacle.emplace_back(42.0, 17.0);
    a_obstacle.emplace_back(42.0, 12.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(48.0, 12.0);
    a_obstacle.emplace_back(50.0, 12.0);
    a_obstacle.emplace_back(50.0, 16.0);
    a_obstacle.emplace_back(48.0, 16.0);
    a_obstacle.emplace_back(48.0, 12.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(51.0, 12.0);
    a_obstacle.emplace_back(53.0, 12.0);
    a_obstacle.emplace_back(53.0, 17.0);
    a_obstacle.emplace_back(51.0, 17.0);
    a_obstacle.emplace_back(51.0, 12.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(58.0, 13.0);
    a_obstacle.emplace_back(60.0, 13.0);
    a_obstacle.emplace_back(60.0, 18.0);
    a_obstacle.emplace_back(58.0, 18.0);
    a_obstacle.emplace_back(58.0, 13.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(61.0, 12.0);
    a_obstacle.emplace_back(63.0, 12.0);
    a_obstacle.emplace_back(63.0, 16.0);
    a_obstacle.emplace_back(61.0, 16.0);
    a_obstacle.emplace_back(61.0, 12.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(64.0, 13.0);
    a_obstacle.emplace_back(66.0, 13.0);
    a_obstacle.emplace_back(66.0, 17.0);
    a_obstacle.emplace_back(64.0, 17.0);
    a_obstacle.emplace_back(64.0, 13.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(70.0, 11.0);
    a_obstacle.emplace_back(72.0, 11.0);
    a_obstacle.emplace_back(72.0, 15.0);
    a_obstacle.emplace_back(70.0, 15.0);
    a_obstacle.emplace_back(70.0, 11.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(73.0, 12.0);
    a_obstacle.emplace_back(75.0, 12.0);
    a_obstacle.emplace_back(75.0, 16.0);
    a_obstacle.emplace_back(73.0, 16.0);
    a_obstacle.emplace_back(73.0, 12.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(76.0, 12.0);
    a_obstacle.emplace_back(78.0, 12.0);
    a_obstacle.emplace_back(78.0, 17.0);
    a_obstacle.emplace_back(76.0, 17.0);
    a_obstacle.emplace_back(76.0, 12.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(3.0, 23.0);
    a_obstacle.emplace_back(5.0, 23.0);
    a_obstacle.emplace_back(5.0, 28.0);
    a_obstacle.emplace_back(3.0, 28.0);
    a_obstacle.emplace_back(3.0, 23.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(6.0, 22.0);
    a_obstacle.emplace_back(8.0, 22.0);
    a_obstacle.emplace_back(8.0, 26.0);
    a_obstacle.emplace_back(6.0, 26.0);
    a_obstacle.emplace_back(6.0, 22.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(9.0, 23.0);
    a_obstacle.emplace_back(11.0, 23.0);
    a_obstacle.emplace_back(11.0, 27.0);
    a_obstacle.emplace_back(9.0, 27.0);
    a_obstacle.emplace_back(9.0, 23.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(15.0, 23.0);
    a_obstacle.emplace_back(17.0, 23.0);
    a_obstacle.emplace_back(17.0, 27.0);
    a_obstacle.emplace_back(15.0, 27.0);
    a_obstacle.emplace_back(15.0, 23.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(18.0, 24.0);
    a_obstacle.emplace_back(20.0, 24.0);
    a_obstacle.emplace_back(20.0, 28.0);
    a_obstacle.emplace_back(18.0, 28.0);
    a_obstacle.emplace_back(18.0, 24.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(21.0, 24.0);
    a_obstacle.emplace_back(23.0, 24.0);
    a_obstacle.emplace_back(23.0, 29.0);
    a_obstacle.emplace_back(21.0, 29.0);
    a_obstacle.emplace_back(21.0, 24.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(24.0, 23.0);
    a_obstacle.emplace_back(26.0, 23.0);
    a_obstacle.emplace_back(26.0, 27.0);
    a_obstacle.emplace_back(24.0, 27.0);
    a_obstacle.emplace_back(24.0, 23.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(30.0, 24.0);
    a_obstacle.emplace_back(32.0, 24.0);
    a_obstacle.emplace_back(32.0, 29.0);
    a_obstacle.emplace_back(30.0, 29.0);
    a_obstacle.emplace_back(30.0, 24.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(33.0, 23.0);
    a_obstacle.emplace_back(35.0, 23.0);
    a_obstacle.emplace_back(35.0, 27.0);
    a_obstacle.emplace_back(33.0, 27.0);
    a_obstacle.emplace_back(33.0, 23.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(36.0, 24.0);
    a_obstacle.emplace_back(38.0, 24.0);
    a_obstacle.emplace_back(38.0, 28.0);
    a_obstacle.emplace_back(36.0, 28.0);
    a_obstacle.emplace_back(36.0, 24.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(47.0, 23.0);
    a_obstacle.emplace_back(49.0, 23.0);
    a_obstacle.emplace_back(49.0, 28.0);
    a_obstacle.emplace_back(47.0, 28.0);
    a_obstacle.emplace_back(47.0, 23.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(50.0, 22.0);
    a_obstacle.emplace_back(52.0, 22.0);
    a_obstacle.emplace_back(52.0, 26.0);
    a_obstacle.emplace_back(50.0, 26.0);
    a_obstacle.emplace_back(50.0, 22.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(53.0, 23.0);
    a_obstacle.emplace_back(55.0, 23.0);
    a_obstacle.emplace_back(55.0, 27.0);
    a_obstacle.emplace_back(53.0, 27.0);
    a_obstacle.emplace_back(53.0, 23.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(56.0, 22.0);
    a_obstacle.emplace_back(58.0, 22.0);
    a_obstacle.emplace_back(58.0, 27.0);
    a_obstacle.emplace_back(56.0, 27.0);
    a_obstacle.emplace_back(56.0, 22.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(59.0, 21.0);
    a_obstacle.emplace_back(61.0, 21.0);
    a_obstacle.emplace_back(61.0, 25.0);
    a_obstacle.emplace_back(59.0, 25.0);
    a_obstacle.emplace_back(59.0, 21.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(62.0, 22.0);
    a_obstacle.emplace_back(64.0, 22.0);
    a_obstacle.emplace_back(64.0, 26.0);
    a_obstacle.emplace_back(62.0, 26.0);
    a_obstacle.emplace_back(62.0, 22.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(65.0, 22.0);
    a_obstacle.emplace_back(67.0, 22.0);
    a_obstacle.emplace_back(67.0, 27.0);
    a_obstacle.emplace_back(65.0, 27.0);
    a_obstacle.emplace_back(65.0, 22.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(11.0, 33.0);
    a_obstacle.emplace_back(13.0, 33.0);
    a_obstacle.emplace_back(13.0, 38.0);
    a_obstacle.emplace_back(11.0, 38.0);
    a_obstacle.emplace_back(11.0, 33.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(14.0, 32.0);
    a_obstacle.emplace_back(16.0, 32.0);
    a_obstacle.emplace_back(16.0, 36.0);
    a_obstacle.emplace_back(14.0, 36.0);
    a_obstacle.emplace_back(14.0, 32.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(20.0, 33.0);
    a_obstacle.emplace_back(22.0, 33.0);
    a_obstacle.emplace_back(22.0, 38.0);
    a_obstacle.emplace_back(20.0, 38.0);
    a_obstacle.emplace_back(20.0, 33.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(26.0, 33.0);
    a_obstacle.emplace_back(28.0, 33.0);
    a_obstacle.emplace_back(28.0, 38.0);
    a_obstacle.emplace_back(26.0, 38.0);
    a_obstacle.emplace_back(26.0, 33.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(29.0, 32.0);
    a_obstacle.emplace_back(31.0, 32.0);
    a_obstacle.emplace_back(31.0, 36.0);
    a_obstacle.emplace_back(29.0, 36.0);
    a_obstacle.emplace_back(29.0, 32.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(32.0, 33.0);
    a_obstacle.emplace_back(34.0, 33.0);
    a_obstacle.emplace_back(34.0, 37.0);
    a_obstacle.emplace_back(32.0, 37.0);
    a_obstacle.emplace_back(32.0, 33.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(35.0, 33.0);
    a_obstacle.emplace_back(37.0, 33.0);
    a_obstacle.emplace_back(37.0, 38.0);
    a_obstacle.emplace_back(35.0, 38.0);
    a_obstacle.emplace_back(35.0, 33.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(41.0, 33.0);
    a_obstacle.emplace_back(43.0, 33.0);
    a_obstacle.emplace_back(43.0, 38.0);
    a_obstacle.emplace_back(41.0, 38.0);
    a_obstacle.emplace_back(41.0, 33.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(47.0, 33.0);
    a_obstacle.emplace_back(49.0, 33.0);
    a_obstacle.emplace_back(49.0, 37.0);
    a_obstacle.emplace_back(47.0, 37.0);
    a_obstacle.emplace_back(47.0, 33.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(50.0, 33.0);
    a_obstacle.emplace_back(52.0, 33.0);
    a_obstacle.emplace_back(52.0, 38.0);
    a_obstacle.emplace_back(50.0, 38.0);
    a_obstacle.emplace_back(50.0, 33.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(56.0, 32.0);
    a_obstacle.emplace_back(58.0, 32.0);
    a_obstacle.emplace_back(58.0, 36.0);
    a_obstacle.emplace_back(56.0, 36.0);
    a_obstacle.emplace_back(56.0, 32.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(59.0, 33.0);
    a_obstacle.emplace_back(61.0, 33.0);
    a_obstacle.emplace_back(61.0, 37.0);
    a_obstacle.emplace_back(59.0, 37.0);
    a_obstacle.emplace_back(59.0, 33.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(23.0, 43.0);
    a_obstacle.emplace_back(25.0, 43.0);
    a_obstacle.emplace_back(25.0, 48.0);
    a_obstacle.emplace_back(23.0, 48.0);
    a_obstacle.emplace_back(23.0, 43.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(26.0, 42.0);
    a_obstacle.emplace_back(28.0, 42.0);
    a_obstacle.emplace_back(28.0, 46.0);
    a_obstacle.emplace_back(26.0, 46.0);
    a_obstacle.emplace_back(26.0, 42.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(29.0, 43.0);
    a_obstacle.emplace_back(31.0, 43.0);
    a_obstacle.emplace_back(31.0, 47.0);
    a_obstacle.emplace_back(29.0, 47.0);
    a_obstacle.emplace_back(29.0, 43.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(32.0, 42.0);
    a_obstacle.emplace_back(34.0, 42.0);
    a_obstacle.emplace_back(34.0, 47.0);
    a_obstacle.emplace_back(32.0, 47.0);
    a_obstacle.emplace_back(32.0, 42.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(35.0, 41.0);
    a_obstacle.emplace_back(37.0, 41.0);
    a_obstacle.emplace_back(37.0, 45.0);
    a_obstacle.emplace_back(35.0, 45.0);
    a_obstacle.emplace_back(35.0, 41.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(38.0, 42.0);
    a_obstacle.emplace_back(40.0, 42.0);
    a_obstacle.emplace_back(40.0, 46.0);
    a_obstacle.emplace_back(38.0, 46.0);
    a_obstacle.emplace_back(38.0, 42.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(41.0, 42.0);
    a_obstacle.emplace_back(43.0, 42.0);
    a_obstacle.emplace_back(43.0, 47.0);
    a_obstacle.emplace_back(41.0, 47.0);
    a_obstacle.emplace_back(41.0, 42.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(58.0, 42.0);
    a_obstacle.emplace_back(60.0, 42.0);
    a_obstacle.emplace_back(60.0, 47.0);
    a_obstacle.emplace_back(58.0, 47.0);
    a_obstacle.emplace_back(58.0, 42.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(61.0, 41.0);
    a_obstacle.emplace_back(63.0, 41.0);
    a_obstacle.emplace_back(63.0, 45.0);
    a_obstacle.emplace_back(61.0, 45.0);
    a_obstacle.emplace_back(61.0, 41.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(67.0, 41.0);
    a_obstacle.emplace_back(69.0, 41.0);
    a_obstacle.emplace_back(69.0, 46.0);
    a_obstacle.emplace_back(67.0, 46.0);
    a_obstacle.emplace_back(67.0, 41.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(70.0, 40.0);
    a_obstacle.emplace_back(72.0, 40.0);
    a_obstacle.emplace_back(72.0, 44.0);
    a_obstacle.emplace_back(70.0, 44.0);
    a_obstacle.emplace_back(70.0, 40.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(73.0, 41.0);
    a_obstacle.emplace_back(75.0, 41.0);
    a_obstacle.emplace_back(75.0, 45.0);
    a_obstacle.emplace_back(73.0, 45.0);
    a_obstacle.emplace_back(73.0, 41.0);
    obstacles_list.emplace_back(a_obstacle);

    a_obstacle.clear();
    a_obstacle.emplace_back(76.0, 41.0);
    a_obstacle.emplace_back(78.0, 41.0);
    a_obstacle.emplace_back(78.0, 46.0);
    a_obstacle.emplace_back(76.0, 46.0);
    a_obstacle.emplace_back(76.0, 41.0);
    obstacles_list.emplace_back(a_obstacle);

    // has_obstacle = true;
    // visualizationObstacles(obstacles_list_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_optimizer_node");
    ros::NodeHandle nh("~");
    obs_pub = nh.advertise<visualization_msgs::MarkerArray>("obsmarkers", 1);
    path_pub = nh.advertise<visualization_msgs::MarkerArray>("path", 1);
    ros::Subscriber startpoint_sub = nh.subscribe("/initialpose", 1, StartpointCallback);
    ros::Subscriber goalpoint_sub = nh.subscribe("/move_base_simple/goal", 1, GoalpointCallback);

    setObs();
    ros::Rate r(10);
    while(nh.ok()){
        ros::spinOnce();
        visualizationObstacles(obstacles_list);
        r.sleep();
    }
    
    return 0;
}