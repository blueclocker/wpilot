#include "optimizer/optimizer.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "osmmap/cubic_spline.h"
#include "osmmap/Pathpoints.h"
#include <fstream>

// 变量
ros::Publisher obs_pub;
ros::Publisher path_pub;
ros::Publisher map_pub;
ros::Publisher result_pub;
double sx;
double sy;
double sphi;
double sv;
double ex;
double ey;
double ephi;
bool has_start;
bool has_end;
optimizer::VehicleParam vehicle_param;
optimizer::PlannerResult result;
optimizer::PlannerOpenSpaceConfig planner_open_space_config_;
std::unique_ptr<optimizer::UnionPlanner> hybrid_test_ = 
std::unique_ptr<optimizer::UnionPlanner>(new optimizer::UnionPlanner(planner_open_space_config_));
// std::vector<double> XYbounds{0.0, 80.0, 0.0, 50.0};
std::vector<double> XYbounds{0.0, 50.0, -4.0, 4.0};
std::vector<std::vector<optimizer::Vec2d>> obstacles_list;
std::vector<optimizer::Vec2d> globalpath;
std::vector<double> start_state(5); //x, y, z, yaw, v


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
    obs.color.r = 0.0;
    obs.scale.x = 0.2;
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

void visualizationMap()
{
    visualization_msgs::MarkerArray map_markers;
    visualization_msgs::Marker map_marker;
    map_marker.header.frame_id = "map";
    map_marker.header.stamp = ros::Time::now();
    map_marker.action = visualization_msgs::Marker::ADD;
    map_marker.type = visualization_msgs::Marker::LINE_STRIP;
    map_marker.ns = "map";
    // obs.lifetime = ros::Duration(0.1);
    map_marker.pose.orientation.w = 1.0;
    map_marker.color.a = 1.0;
    map_marker.color.r = 0.0;
    map_marker.scale.x = 0.2;
    geometry_msgs::Point a;
    
    // 左边界
    map_marker.id = 0;
    a.x = XYbounds[0];
    a.y = XYbounds[3];
    map_marker.points.push_back(a);
    a.x = XYbounds[1];
    map_marker.points.push_back(a);
    map_markers.markers.push_back(map_marker);

    // 右边界
    map_marker.id = 1;
    a.x = XYbounds[0];
    a.y = XYbounds[2];
    map_marker.points.clear();
    map_marker.points.push_back(a);
    a.x = XYbounds[1];
    map_marker.points.push_back(a);
    map_markers.markers.push_back(map_marker);

    // 中心线
    map_marker.id = 2;
    a.x = XYbounds[0];
    a.y = (XYbounds[3] + XYbounds[2]) / 2.0;
    map_marker.points.clear();
    map_marker.scale.x = 0.1;
    map_marker.points.push_back(a);
    a.x = XYbounds[1];
    map_marker.points.push_back(a);
    map_markers.markers.push_back(map_marker);

    map_pub.publish(map_markers);
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
    m.color.r = 1.0;
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
    double ego_length = vehicle_param.length;
    double shift_distance =
        ego_length / 2.0 - vehicle_param.back_edge_to_center;

    visualization_msgs::Marker path_point;
    path_point.header.frame_id = "map";
    path_point.header.stamp = ros::Time::now();
    path_point.type = visualization_msgs::Marker::SPHERE_LIST;
    path_point.action = visualization_msgs::Marker::ADD;
    // vehicle.id = static_cast<int>(i / vehicle_interval);
    path_point.ns = "path_points";
    path_point.scale.x = 0.3;
    path_point.scale.y = 0.3;
    path_point.scale.z = 0.3;
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
        vehicle.pose.position.x = a.x + shift_distance * cos(path.phi[i]);
        vehicle.pose.position.y = a.y + shift_distance * sin(path.phi[i]);
        vehicle.pose.position.z = 0.0;
        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path.phi[i]);
        paths.markers.push_back(vehicle);
    }

    paths.markers.push_back(m);
    paths.markers.push_back(path_point);
    path_pub.publish(paths);
}

void PubResult()
{
    osmmap::Pathpoints content;
    content.header.frame_id = "result";
    std::string file_path = "/home/wp/wpollo/src/lanelet/osmmap/result.csv";
    std::ofstream outFile(file_path, std::ios::out);
    if (outFile.is_open())
    {
        outFile << "time," <<
                "x," <<
                "y," <<
                "phi," <<
                "v," <<
                "steer," <<
                "a" << std::endl;
        for(int i = 0; i < result.x.size(); ++i)
        {
            content.header.stamp = ros::Time::now();
            content.x = result.x[i];
            content.y = result.y[i];
            content.phi = result.phi[i];
            content.v = result.v[i];
            content.steer = result.steer[i];
            if(i+1 < result.x.size()){
                content.a = result.v[i+1] - result.v[i];
            }else{
                content.a = result.v[i] - result.v[i-1]; 
            }
            outFile << std::to_string(i) << "," <<
                       std::to_string(content.x) << "," <<
                       std::to_string(content.y) << "," <<
                       std::to_string(content.phi) << "," <<
                       std::to_string(content.v) << "," <<
                       std::to_string(content.steer) << "," <<
                       std::to_string(content.a) << std::endl;

            result_pub.publish(content);
        }
        outFile.close();
    }
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
        bool find = hybrid_test_->Plan(sx, sy, sphi, 10.0, ex, ey, ephi, 
                                       XYbounds, obstacles_list, globalpath, &result);
        if(!find) return;
        std::cout << "the number of find path is " << result.x.size() << std::endl;
        visualizationPath(result);
        PubResult();
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
        bool find = hybrid_test_->Plan(sx, sy, sphi, 10.0, ex, ey, ephi, 
                                       XYbounds, obstacles_list, globalpath, &result);
        if(!find) return;
        std::cout << "the number of find path is " << result.x.size() << std::endl;
        visualizationPath(result);
        PubResult();
    }
}

void PathCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
    ROS_INFO("Received current path ...");
    globalpath.clear();

    for(int i = 0; i < msg->points.size(); ++i)
    {
        globalpath.emplace_back(msg->points[i].x, msg->points[i].y);
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    ROS_INFO("Received current odometry in trajectory controller ...");
    std::vector<double> current_pose(3);     // x, y, yaw
    std::vector<double> current_velocity(3); // vx, vy, magnitude
    double current_z = 0;

    geometry_msgs::Quaternion geo_quat = msg->pose.pose.orientation;

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(geo_quat, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    current_pose[0] = msg->pose.pose.position.x;
    current_pose[1] = msg->pose.pose.position.y;
    current_pose[2] = yaw;

    current_velocity[0] = msg->twist.twist.linear.x;
    current_velocity[1] = msg->twist.twist.linear.y;
    current_velocity[2] = std::hypot(current_velocity[0], current_velocity[1]);

    current_z = msg->pose.pose.position.z;

    start_state[0] = current_pose[0];
    start_state[1] = current_pose[1];
    start_state[2] = current_z;
    start_state[3] = current_pose[2];
    start_state[4] = current_velocity[2];
}

void setGlobalpath()
{
    globalpath.clear();
    std::vector<map::centerway::CenterPoint3D> pathnode;
    for(double i = 0.0; i < 51; i += 5.0){
        pathnode.emplace_back(i, 0.0);
    }
    plan::Spline2D csp_obj(pathnode);
    for(double i = 0; i < csp_obj.s.back(); i += 0.2)
    {
        std::array<double, 3> point = csp_obj.calc_postion(i);
        globalpath.emplace_back(point[0], point[1]);
    }
    //
}

void setCruise()
{
    ROS_INFO("set obstacles!");
    obstacles_list.clear();
    std::vector<optimizer::Vec2d> a_obstacle;
    a_obstacle.emplace_back(20.0, -3.5);
    a_obstacle.emplace_back(25.0, -3.5);
    a_obstacle.emplace_back(25.0, -0.5);
    a_obstacle.emplace_back(20.0, -0.5);
    a_obstacle.emplace_back(20.0, -3.5);
    obstacles_list.emplace_back(a_obstacle);
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
    map_pub = nh.advertise<visualization_msgs::MarkerArray>("map", 1);
    result_pub = nh.advertise<osmmap::Pathpoints>("result", 1);
    ros::Subscriber startpoint_sub = nh.subscribe("/initialpose", 1, StartpointCallback);
    ros::Subscriber odom_sub = nh.subscribe("/carla/ego_vehicle/odometry", 1, odomCallback);
    ros::Subscriber goalpoint_sub = nh.subscribe("/move_base_simple/goal", 1, GoalpointCallback);
    ros::Subscriber globalpath_sub = nh.subscribe("/navagation_node/golbalpath_info", 1, PathCallback);

    // setObs();
    setCruise();
    setGlobalpath();
    ros::Rate r(10);
    while(nh.ok()){
        ros::spinOnce();
        visualizationObstacles(obstacles_list);
        visualizationMap();
        r.sleep();
    }
    
    return 0;
}