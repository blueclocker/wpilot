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

ros::Publisher perfect_control_pub;
std::vector<double> current_position(5); //x, y, z, yaw, v
std::vector<double> next_position(5); // x, y, z, yaw, v
std::deque<double> m_buffer_pid;
double prev_throttle = 0;
double v_error_previous = 0;
double alpha_previous = 0;

void trajectoryCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
    ROS_INFO("Received current path ...");
    next_position[4] = 0;
    if(msg->points.size() < 2) return;
    double lookahead = current_position[4];
    if(lookahead < 2.0) lookahead = 2.0;

    double sum_length = 0.2;
    double next_x = msg->points[1].x;
    double next_y = msg->points[1].y;
    double next_z = msg->points[1].z;
    bool is_stop = true;
    for(int i = 2; i < msg->points.size(); ++i){
        if(sum_length > lookahead) {
            is_stop = false;
            break;
        }
        next_x = msg->points[i].x;
        next_y = msg->points[i].y;
        next_z = msg->points[i].z;
        sum_length += 0.2;
    }

    double diff_x = next_x - msg->points[0].x;
    double diff_y = next_y - msg->points[0].y;
    double yaw = std::atan2(diff_y, diff_x);

    next_position[0] = next_x;
    next_position[1] = next_y;
    next_position[2] = next_z;
    next_position[3] = yaw;
    next_position[4] = 5.0;
    if(is_stop) next_position[4] = 0;

    // std::cout << "cur_x: " << current_position[0] << ", cur_y: " << current_position[1] << std::endl;
    // std::cout << "first_x: " << msg->points[0].x << ", first_y: " << msg->points[0].y << std::endl;
    // std::cout << "next_x: " << next_x << ", next_y: " << next_y << std::endl;
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

    current_position[0] = current_pose[0];
    current_position[1] = current_pose[1];
    current_position[2] = current_z;
    current_position[3] = current_pose[2];
    current_position[4] = current_velocity[2];
}

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

void calc_command(){
    double steering = 0;
    double throttle = 0;
    double brake = 0;

    //steer
    double kvf = 1.0;       // CONSTANT TO BE MOVED TO CONSTRUCTOR
    double wheelbase = 2.8; // CONSTANT TO BE MOVED TO CONSTRUCTOR
    double kpp = 0.1;       // CONSTANT TO BE MOVED TO CONSTRUCTOR
    std::vector<double> v1 = {next_position[0]-current_position[0], next_position[1] - current_position[1]};
    double lookahead_distance = std::sqrt(v1[0]*v1[0] + v1[1]*v1[1]);

    double alpha = current_position[3] - next_position[3];
    // std::cout << "alpha: " << alpha << ", yaw: " << current_position[3] << std::endl;
    // std::cout << "lookahead distance: " << lookahead_distance << std::endl;
    // steering = get_steering_direction(v1, v2) *
    //            atan((2 * wheelbase * sin(alpha)) / lookahead_distance);
    steering = atan((2 * wheelbase * sin(alpha)) / lookahead_distance);

    //throttle
    double kp = 0.5;  // CONSTANT TO BE MOVED TO CONSTRUCTOR
    double ki = 0.02; // CONSTANT TO BE MOVED TO CONSTRUCTOR
    double kd = 0.05; // CONSTANT TO BE MOVED TO CONSTRUCTOR
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
    if (next_position[4] < 0.01) {
      throttle = 0;
      steering = 0;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "perfect_control_node");
    ros::NodeHandle nh("~");
    perfect_control_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1);
    ros::Subscriber odom_sub = nh.subscribe("/carla/ego_vehicle/odometry", 1, odomCallback);
    ros::Subscriber trajectory_sub = nh.subscribe("/navagation_node/golbalpath_info", 1, trajectoryCallback);
    ros::Rate r(10);

    while(nh.ok())
    {
        std::cout << "------------------------------------------" << std::endl;
        calc_command();
        std::cout << "------------------------------------------" << std::endl;
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}