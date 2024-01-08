#include <ros/ros.h>
#include <iostream>
#include <termios.h>
#include <cmath>
#include <std_msgs/Float32.h>
#include <path_tracking/vehicle_can_r.h>
#include <path_tracking/vehicle_can_s.h>
#include <path_tracking/path.h>
#include "fsd_common_msgs/Comb.h"
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <vector>
#include <fstream>  
#include <sstream>  
#include <string>
      
using namespace std;  
      
     
float Vehicle_speed_now;
float DriveMode_state_now;
float target_Vehicle_speed = 8;
float speed_error_0;
float speed_error_1;
float speed_error_2;
float speed_error_3;
float kp = 3;
float ki = 0.01;
float kd = 0;
float kk=0.1;
float vehicle_x;
float vehicle_y;
float vehicle_heading;
float target_x;
float target_y;
float target_heading;
float i;
float Longitude_init = 116.3108713;
float Lattitude_init = 39.9558476;
float pi=4*atan(1);
float l_error;
float h_error;
int minPosition;
int m=700;
int n=3;

Eigen::MatrixXf path(m,n);
Eigen::VectorXf dis(m);


path_tracking::vehicle_can_s msg_2;
path_tracking::path msg_1;

void ctlCallback_1(const path_tracking::vehicle_can_r& msg)
{   
	Vehicle_speed_now=msg.Vehicle_speed;
	DriveMode_state_now=msg.DriveMode_state;	
}
void ctlCallback_2(const fsd_common_msgs::Comb& msg)
{   
	vehicle_x=(msg.Longitude-Longitude_init)/0.00001141;
	vehicle_y=(msg.Lattitude-Lattitude_init)/0.00000899;
	vehicle_heading=msg.Heading+90;
	float mindis=100;
	for( int b = 0; b < m; b = b + 1 )
  	{
    		dis(b)=sqrt((vehicle_x-path(b,0))*(vehicle_x-path(b,0))+(vehicle_y-path(b,1))*(vehicle_y-path(b,1)));
	
  	}
	int c=0;
	while (c < m)
	{
		if (mindis>dis[c])
		{
			mindis=dis[c];
			minPosition=c;
		}
		c=c+1;
	//ROS_INFO("c=%d,dis=%f,p=%d",c,mindis,minPosition);
	}
	target_x=path(minPosition+2,0);
	target_y=path(minPosition+2,1);
	target_heading=path(minPosition+5,2);
	l_error=target_y-vehicle_y;
	h_error=target_heading-vehicle_heading;
}    

void nothing_Callback(const std_msgs::Float32& msg)
{   
} 

string Trim(string& str)
{
	//str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置
	str.erase(0,str.find_first_not_of(" \t\r\n"));
	str.erase(str.find_last_not_of(" \t\r\n") + 1);
	return str;
}
 
int main(int argc, char** argv) {

	ifstream fin("/home/a/zh_ws/bag/path_2.csv"); //打开文件流操做
	string line; 
	int k=-1;
	while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
		istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		vector<string> fields; //声明一个字符串向量
		string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
		}
		string path_x = Trim(fields[1]); //清除掉向量fields中第一个元素的无效字符，并赋值给变量name
		string path_y = Trim(fields[2]); //清除掉向量fields中第二个元素的无效字符，并赋值给变量age
		string path_heading = Trim(fields[3]); //清除掉向量fields中第三个元素的无效字符，并赋值给变量birthday
		//cout <<"处理以后的字符串："<< path_x << "\t" << path_y << "\t" << path_heading << endl; 
		if (k>-1)
		{
			path(k,0)=atof(path_x.c_str());
			path(k,1)=atof(path_y.c_str());
			path(k,2)=atof(path_heading.c_str());
		}
		k=k+1;
	}

  ros::init(argc, argv, "path_track_stanley");
  ros::NodeHandle nh;
  
  ros::Publisher pub_vehicle_can_s;
  ros::Publisher pub_path;

  /*---Subscriber---*/
  ros::Subscriber sub_vehicle_can_r;
  ros::Subscriber sub_gps_data;
  ros::Subscriber sub_nothing; 

  /*---Publisher---*/
  pub_vehicle_can_s = nh.advertise<path_tracking::vehicle_can_s>("vehicle_can_s", 1);
  pub_path = nh.advertise<path_tracking::path>("path", 1);

  /*---Sending Data---*/
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    sub_vehicle_can_r = nh.subscribe("vehicle_can_r", 1, ctlCallback_1);
    sub_gps_data = nh.subscribe("comb", 1, ctlCallback_2);
    sub_nothing = nh.subscribe("nothing", 1, nothing_Callback);
    msg_2.Enable=1;
    msg_2.Acc_valid = 1;	
    msg_1.x=vehicle_x;
    msg_1.y=vehicle_y;
    msg_1.heading=vehicle_heading;
    
    if (DriveMode_state_now == 1 || DriveMode_state_now ==3 )
    {
	i = 1;
	msg_2.Gear_request = 1;
	msg_2.Steering_valid=1;
	msg_2.Tir_speed_cmd=10;
    }
    else 
    {
	i = 0;
	msg_2.Gear_request = 0;
    }
    
    speed_error_1 = target_Vehicle_speed - Vehicle_speed_now ;
    speed_error_2 = speed_error_2*i + speed_error_1;
    speed_error_3 = speed_error_0 - speed_error_1;
    speed_error_0 = speed_error_1;
    msg_2.Acc_cmd = kp*speed_error_1 + ki*speed_error_2 + kd*speed_error_3;
    if (msg_2.Acc_cmd>100)
	{
		msg_2.Acc_cmd=100;
	}
    msg_2.Tir_angle_cmd= 0.8*h_error+atan(kk*l_error*3.6/target_Vehicle_speed)/pi*180;
    if (msg_2.Tir_angle_cmd<-30)
	{
		msg_2.Tir_angle_cmd=-30;
	}
    if (msg_2.Tir_angle_cmd>30)
	{
		msg_2.Tir_angle_cmd=30;
	}
    ROS_INFO("x=%f,y=%f,tx=%f,ty=%f,e=%f,a=%f,s=%f",vehicle_x,vehicle_y,target_x,l_error,h_error,msg_2.Acc_cmd,msg_2.Tir_angle_cmd);
    // Publish Data
    pub_vehicle_can_s.publish(msg_2);
    pub_path.publish(msg_1);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
