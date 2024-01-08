#include <ros/ros.h>
#include <iostream>
#include <termios.h>
#include <cmath>
#include <std_msgs/Float32.h>
#include <path_tracking/vehicle_can_r.h>
#include <path_tracking/vehicle_can_s.h>
#include <path_tracking/path.h>
#include "osmmap/CarState.h"
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose2D.h>
#include <lgsvl_msgs/VehicleControlData.h>

using namespace std;

float Vehicle_speed_now;
float DriveMode_state_now;
float target_Vehicle_speed = 4;
float speed_error_0;
float speed_error_1;
float speed_error_2;
float speed_error_3;
float kp = 6;
float ki = 0.01;
float kd = 0;
float kk = 0.1;

double vehicle_heading;
double target_x;
double target_y;
double target_heading;
double target_yaw;

float i;
float pi = 4 * atan(1);
float l_error;
float h_error;
float h_error_2;
int num;

Eigen::VectorXf dis(30);

path_tracking::vehicle_can_s msg_2;
path_tracking::path msg_1;
lgsvl_msgs::VehicleControlData vehicle_data;

void ctlCallback_1(const path_tracking::vehicle_can_r &msg)
{
	Vehicle_speed_now = msg.Vehicle_speed;
	DriveMode_state_now = msg.DriveMode_state;
}

// void ctlCallback_3(const fsd_common_msgs::Comb& msg)
// {
// 	vehicle_heading=msg.Heading+90;
// }

void vehiclestatecallback(const osmmap::CarState::ConstPtr &msg)
{
	Vehicle_speed_now = msg->linear.x;
	vehicle_heading = msg->heading / M_PI * 180;
	DriveMode_state_now = 1;
}

void ctlCallback_2(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
	for (int b = 0; b < msg->markers.size(); b = b + 1)
	{
		if (b >= 30)
			break;
		dis(b) = sqrt((msg->markers[0].points[0].x - msg->markers[0].points[b].x) * (msg->markers[0].points[0].x - msg->markers[0].points[b].x) + (msg->markers[0].points[0].y - msg->markers[0].points[b].y) * (msg->markers[0].points[0].y - msg->markers[0].points[b].y));
		if (dis(b) > 1.5)
		{
			num = b;
			break;
		}
	}
	target_x = msg->markers[0].points[num].x;
	target_y = msg->markers[0].points[num].y;
	target_heading = atan2((target_y - msg->markers[0].points[0].y), (target_x - msg->markers[0].points[0].x)) * 180 / pi;
	h_error_2 = target_heading - vehicle_heading;
	if (h_error_2 > 180)
	{
		h_error_2 = h_error_2 - 360;
	}
	else if (h_error_2 < -180)
	{
		h_error_2 = h_error_2 + 360;
	}
	target_x = dis(num) * cos(h_error_2 / 180 * pi);
	target_y = dis(num) * sin(h_error_2 / 180 * pi);
	l_error = target_y;

	target_yaw = msg->markers[0].points[num].z * 180 / pi;
	h_error = target_yaw - vehicle_heading;
	if (h_error > 180)
	{
		h_error = h_error - 360;
	}
	else if (h_error < -180)
	{
		h_error = h_error + 360;
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "path_track_stanley_new");
	ros::NodeHandle nh;

	ros::Publisher pub_vehicle_can_s;
	ros::Publisher pub_path;
	ros::Publisher pub_lgsvl;

	/*---Subscriber---*/
	//ros::Subscriber sub_vehicle_can_r;
	ros::Subscriber sub_motionplan_data;
	//ros::Subscriber sub_gps_data;
	ros::Subscriber sub_carstate;

	/*---Publisher---*/
	pub_vehicle_can_s = nh.advertise<path_tracking::vehicle_can_s>("vehicle_can_s", 1);
	pub_path = nh.advertise<path_tracking::path>("path", 1);
	pub_lgsvl = nh.advertise<lgsvl_msgs::VehicleControlData>("/vehicle_cmd", 1);
	vehicle_data.header.frame_id = "map";

	/*---Sending Data---*/
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		// sub_vehicle_can_r = nh.subscribe("vehicle_can_r", 1, ctlCallback_1);
		// sub_gps_data = nh.subscribe("comb", 1, ctlCallback_3);
		sub_motionplan_data = nh.subscribe("/planning/frenet_path", 1, ctlCallback_2);
		sub_carstate = nh.subscribe("/navagation_node/carstate_info", 1, vehiclestatecallback);
		msg_2.Enable = 1;
		msg_2.Acc_valid = 1;
		msg_1.x = target_x;
		msg_1.y = target_y;
		msg_1.heading = vehicle_heading;
		float t = DriveMode_state_now;
		msg_1.Drivemodestate = t;

		if (DriveMode_state_now == 1)
		{
			i = 1;
			msg_2.Gear_request = 1;
			msg_2.Steering_valid = 1;
			msg_2.Tir_speed_cmd = 10;
			ROS_INFO("Autodriving--------Be careful!");
		}
		else if (DriveMode_state_now == 3)
		{
			i = 1;
			msg_2.Gear_request = 1;
			msg_2.Steering_valid = 1;
			msg_2.Tir_speed_cmd = 10;
			ROS_INFO("Autodriving--------Be careful!");
		}
		else
		{
			i = 0;
			msg_2.Gear_request = 0;
		}

		speed_error_1 = target_Vehicle_speed - Vehicle_speed_now;
		speed_error_2 = speed_error_2 * i + speed_error_1;
		speed_error_3 = speed_error_0 - speed_error_1;
		speed_error_0 = speed_error_1;
		msg_2.Acc_cmd = kp * speed_error_1 + ki * speed_error_2 + kd * speed_error_3;
		if (msg_2.Acc_cmd > 100)
		{
			msg_2.Acc_cmd = 100;
		}
		msg_2.Tir_angle_cmd = h_error + atan(kk * l_error * 3.6 / target_Vehicle_speed) / pi * 180;
		if (msg_2.Tir_angle_cmd < -30)
		{
			msg_2.Tir_angle_cmd = -30;
		}
		if (msg_2.Tir_angle_cmd > 30)
		{
			msg_2.Tir_angle_cmd = 30;
		}
		msg_1.acc = msg_2.Acc_cmd;
		msg_1.steer = msg_2.Tir_angle_cmd;

		vehicle_data.target_gear = lgsvl_msgs::VehicleControlData::GEAR_DRIVE;
		vehicle_data.acceleration_pct = msg_1.acc > 0 ? msg_1.acc / 100 : 0;
		vehicle_data.braking_pct = msg_1.acc < 0 ? -msg_1.acc / 100 : 0;
		vehicle_data.target_wheel_angle = msg_1.steer / 180 * M_PI;
		vehicle_data.target_wheel_angular_rate = abs(vehicle_data.target_wheel_angle) / 10;
		std::cout << "planning acc: " << vehicle_data.acceleration_pct << " ,brake: " << vehicle_data.braking_pct << std::endl;
		std::cout << "planning angle: " << vehicle_data.target_wheel_angle << " ,angular rate: " << vehicle_data.target_wheel_angular_rate << std::endl;

		// Publish Data
		// pub_vehicle_can_s.publish(msg_2);
		// pub_path.publish(msg_1);
		vehicle_data.header.stamp = ros::Time::now();
		pub_lgsvl.publish(vehicle_data);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
