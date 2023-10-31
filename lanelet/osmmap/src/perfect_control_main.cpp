/*
 * @Author: wpbit
 * @Date: 2023-10-31 16:17:40
 * @LastEditors: wpbit
 * @LastEditTime: 2023-10-31 22:02:34
 * @Description: 
 */
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <termios.h>
#include <iostream>

#define PI_CONST 3.141592653589793238462643383279502884

nav_msgs::Odometry next_position;
ros::Publisher perfect_control_pub;

void trajectoryCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
    if(msg->points.size() < 2) return;
    double next_x = msg->points[1].x;
    double next_y = msg->points[1].y;
    double next_z = msg->points[1].z;
    next_position.pose.pose.position.x = next_x;
    next_position.pose.pose.position.y = next_y;
    next_position.pose.pose.position.z = next_z;

    double diff_x = msg->points[1].x - msg->points[0].x;
    double diff_y = msg->points[1].y - msg->points[0].y;
    double yaw = std::atan2(diff_y, diff_x);

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q;
    q = R;
    next_position.pose.pose.orientation.w = q.w();
    next_position.pose.pose.orientation.x = q.x();
    next_position.pose.pose.orientation.y = q.y();
    next_position.pose.pose.orientation.z = q.z();

    perfect_control_pub.publish(next_position);

    static tf::TransformBroadcaster broadcaster;
    static tf::Transform baselink2map;
    tf::Quaternion qenu2base;
    //GNSS
    qenu2base.setRPY(0.0, 0.0, yaw);
    baselink2map.setRotation(qenu2base);
    baselink2map.setOrigin(tf::Vector3(next_x, next_y, next_z));
    //map->rslidar->base_link
    broadcaster.sendTransform(tf::StampedTransform(baselink2map, ros::Time::now(), "map", "rslidar"));
}

class Keyboard_ctrl {
    struct termios initial_settings, new_settings;
    int peek_character = -1;
    
    public:
    Keyboard_ctrl() { init_keyboard(); };
    ~Keyboard_ctrl() { close_keyboard(); };
    
    int get_keyboard_press_key() {
        kbhit();
        return readch();
        // printf("%02x \n", readch());
    };
    
    private:
    void init_keyboard() {
        tcgetattr(0, &initial_settings);
        new_settings = initial_settings;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        new_settings.c_cc[VEOL] = 1;
        new_settings.c_cc[VEOF] = 2;
        tcsetattr(0, TCSANOW, &new_settings);
    }
    
    void close_keyboard() { tcsetattr(0, TCSANOW, &initial_settings); }
    
    int kbhit() {
        unsigned char ch;
        int nread;
    
        if (peek_character != -1) return 1;
        new_settings.c_cc[VMIN] = 0;
        tcsetattr(0, TCSANOW, &new_settings);
        nread = read(0, &ch, 1);
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0, TCSANOW, &new_settings);
        if (nread == 1) {
            peek_character = ch;
            return 1;
        }
        return 0;
    }
    
    int readch() {
        char ch;
        if (peek_character != -1) {
            ch = peek_character;
            peek_character = -1;
            return ch;
        }
        read(0, &ch, 1);
        return ch;
    }
};

int scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    
    in = getchar();
    
    tcsetattr(0,TCSANOW,&stored_settings);
    return in;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "perfect_control");
    ros::NodeHandle nh("~");
    perfect_control_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    ros::Subscriber trajectory_sub = nh.subscribe("/navagation_node/golbalpath_info", 1, trajectoryCallback);
    bool is_pause = false;
    ros::Rate r(10);
    auto KBC = Keyboard_ctrl();

    while(nh.ok())
    {
        // int key = KBC.get_keyboard_press_key();
        // ROS_INFO("get keyboard press 0x%02X \n", key);

        // int key = scanKeyboard();
        // ROS_INFO("get keyboard press 0x%d \n", key);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
