#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <mutex>

std::mutex mtx;
ros::Publisher perfect_control_pub;
std::vector<std::vector<double>> trajectory;
std::vector<double> goal_position(3);//x, y, z
std::deque<double> m_buffer_pid;
double prev_throttle = 0;
double v_error_previous = 0;
double alpha_previous = 0;
bool arriving = false;

int get_steering_direction(std::vector<double> v1, std::vector<double> v2) {
    double cross_prod = v1[0] * v2[1] - v1[1] * v2[0];
    if (cross_prod >= 0) {
        return -1;
    }
    return 1;
}

double get_alpha(std::vector<double> v1, std::vector<double> v2,
                 double lookahead_distance) {
  double inner_prod = v1[0] * v2[0] + v1[1] * v2[1];
  return acos(inner_prod / lookahead_distance);
}

void calc_command(const std::vector<double> &next_position, const std::vector<double> &current_position){
    // std::vector<double> next_position(5); // x, y, z, yaw, v
    // std::vector<double> current_position(5); // x, y, z, yaw, v
    double steering = 0;
    double throttle = 0;
    double brake = 0;

    //steer
    double wheelbase = 2.8; 
    std::vector<double> v1 = {next_position[0]-current_position[0], next_position[1] - current_position[1]};
    double lookahead_distance = std::sqrt(v1[0]*v1[0] + v1[1]*v1[1]);
    double alpha = current_position[3] - next_position[3];
    steering = atan((2 * wheelbase * sin(alpha)) / lookahead_distance);

    //throttle
    double kp = 0.5;
    double ki = 0.02;
    double kd = 0.05;
    double dt = 0.1;
    double v_error = next_position[4] - current_position[4];
    m_buffer_pid.push_back(v_error);
    if (m_buffer_pid.size() > 10) {
        m_buffer_pid.pop_front();
    }
    double acum_error = 0.0;
    for (int i = 0; i < m_buffer_pid.size(); i++) {
        acum_error += m_buffer_pid[i];
    }

    double k_term = kp * v_error;
    double i_term = ki * acum_error * dt;
    double d_term = kd * (v_error - v_error_previous) / dt;
    throttle =
        std::max(std::min(prev_throttle + k_term + i_term + d_term, 1.0), 0.0);
    prev_throttle = throttle;
    v_error_previous = v_error;

    // command
    double remain_dis = std::sqrt((goal_position[0]-current_position[0])*(goal_position[0]-current_position[0])+
                                  (goal_position[1]-current_position[1])*(goal_position[1]-current_position[1]));
    if(remain_dis < 5.0 || arriving) 
    {
      arriving = true;
      throttle = 0;
      brake = 1;
      ROS_WARN("STOP!");
    }

    carla_msgs::CarlaEgoVehicleControl control_cmd;
    control_cmd.throttle = throttle;
    control_cmd.steer = steering;
    control_cmd.brake = brake;
    control_cmd.hand_brake = false;
    control_cmd.reverse = false;
    control_cmd.gear = 1.0;
    control_cmd.header.frame_id = "ego_control";
    control_cmd.header.stamp = ros::Time::now();
    perfect_control_pub.publish(control_cmd);

    ROS_INFO("steer: %f, throttle: %f, brake: %f", steering, throttle, brake);
}

void trajectoryCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
    ROS_INFO("Received current path ...");

    mtx.lock();
    trajectory.clear();
    for(int i = 0; i < msg->points.size(); ++i)
    {
        trajectory.push_back({msg->points[i].x, msg->points[i].y, msg->points[i].z});
        if(i >= 50) break;
    }
    mtx.unlock();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(trajectory.empty()) return;
    ROS_INFO("Received current odometry in trajectory controller ...");
    std::vector<double> next_position(5); // x, y, z, yaw, v
    std::vector<double> current_position(5); //x, y, z, yaw, v

    geometry_msgs::Quaternion geo_quat = msg->pose.pose.orientation;
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(geo_quat, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_position[0] = msg->pose.pose.position.x;
    current_position[1] = msg->pose.pose.position.y;
    current_position[2] = msg->pose.pose.position.z;
    current_position[3] = yaw;
    current_position[4] = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);

    double lookahead = current_position[4];
    if(lookahead < 2.0) lookahead = 2.0;

    mtx.lock();
    for(int i = 0; i < trajectory.size(); ++i){
        double diff_x = trajectory[i][0] - current_position[0];
        double diff_y = trajectory[i][1] - current_position[1];
        double yaw = std::atan2(diff_y, diff_x);

        next_position[0] = trajectory[i][0];
        next_position[1] = trajectory[i][1];
        next_position[2] = trajectory[i][2];
        next_position[3] = yaw;
        next_position[4] = 5.0;

        if(std::sqrt(diff_x * diff_x + diff_y * diff_y) > lookahead) break;
    }
    if(arriving) next_position[4] = 0.0;
    mtx.unlock();

    calc_command(next_position, current_position);
}

void goalpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("Received goal point position ...");

    goal_position[0] = msg->pose.position.x;
    goal_position[1] = msg->pose.position.y;
    goal_position[2] = msg->pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "perfect_control_node");
    ros::NodeHandle nh("~");
    perfect_control_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1);
    ros::Subscriber odom_sub = nh.subscribe("/carla/ego_vehicle/odometry", 1, odomCallback);
    ros::Subscriber trajectory_sub = nh.subscribe("/navagation_node/golbalpath_info", 1, trajectoryCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, goalpointCallback);
    ros::Rate r(10);

    while(nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}