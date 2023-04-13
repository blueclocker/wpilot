/*
 * @Author: wpbit
 * @Date: 2023-02-10 15:29:21
 * @LastEditors: wpbit
 * @LastEditTime: 2023-02-14 20:28:53
 * @Description: 
 */
#include "navagation/navagation_optimizer.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/Marker.h"

ros::ServiceClient client;
ros::Publisher pub;

//输入当前点的xy坐标和航向角
//返回地图给出的最佳返回点坐标和航向角
//(TODO)目前选点逻辑简单，如果距离终点较近(< 10 m)则直接返回终点位姿，否则返回点姿态与车道方向保持一致
void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_INFO("get current point !");
    osmmap::GetTargetPoint test_msg;
    test_msg.request.Sx = msg->pose.pose.position.x;
    test_msg.request.Sy = msg->pose.pose.position.y;
    test_msg.request.Syaw = tf::getYaw(msg->pose.pose.orientation);
    bool f = client.call(test_msg);

    //visualization
    visualization_msgs::Marker arraw;
    arraw.header.frame_id = "map";
    arraw.header.stamp = ros::Time::now();
    arraw.ns = "target_point";
    arraw.action = visualization_msgs::Marker::ADD;
    arraw.id = 1;
    arraw.type = visualization_msgs::Marker::ARROW;
    arraw.scale.x = 2.0;// 柄直径
    arraw.scale.y = 0.5;// 箭头直径
    arraw.scale.z = 0.5;
    arraw.color.r = 1.0;
    arraw.color.g = 0.0;
    arraw.color.b = 0.0;
    arraw.color.a = 0.8;
    arraw.pose.position.x = test_msg.response.Tx;
    arraw.pose.position.y = test_msg.response.Ty;
    arraw.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, test_msg.response.Tyaw);
    pub.publish(arraw);

    ROS_INFO("x: %f, y: %f", test_msg.response.Tx, test_msg.response.Ty);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_findtargetpoint");
    ros::NodeHandle nh("~");
    client = nh.serviceClient<osmmap::GetTargetPoint>("/navagation_node/gettargetpoint_service");
    ros::Subscriber publishpoint_sub = nh.subscribe("/initialpose", 1, callback);
    pub = nh.advertise<visualization_msgs::Marker>("/test_arraw", 1);

    ros::spin();
    return 0;
}

