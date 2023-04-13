/*
 * @Author: wpbit
 * @Date: 2023-02-14 19:40:57
 * @LastEditors: wpbit
 * @LastEditTime: 2023-02-14 20:45:15
 * @Description: 
 */
#include "navagation/navagation_optimizer.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/MarkerArray.h"

ros::ServiceClient client;
ros::Publisher pub;

//输入当前车在地图坐标系的xy坐标和航向角(实际未用)
//返回的marker包含：车后一点---->该路段的尽头
//(TODO)暂不支持纵向扩展
void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_INFO("get current point!");
    osmmap::GetNeighborLanelet test_msg;
    test_msg.request.Sx = msg->pose.pose.position.x;
    test_msg.request.Sy = msg->pose.pose.position.y;
    test_msg.request.Syaw = tf::getYaw(msg->pose.pose.orientation);
    bool f = client.call(test_msg);

    //visualization
    visualization_msgs::MarkerArray laneletsarray;
    visualization_msgs::Marker laneletl, laneletr;
    laneletl.header.frame_id = "map";
    laneletl.header.stamp = ros::Time::now();
    laneletl.ns = "neighbor_laneletl";
    laneletl.action = visualization_msgs::Marker::ADD;
    laneletl.id = 0;
    laneletl.type = visualization_msgs::Marker::LINE_STRIP;
    laneletl.scale.x = 1.0;
    laneletl.color.r = 1.0;
    laneletl.color.g = 0.0;
    laneletl.color.b = 0.0;
    laneletl.color.a = 0.8;
    laneletl.pose.orientation.w = 1.0;
    laneletl.pose.orientation.x = 0.0;
    laneletl.pose.orientation.y = 0.0;
    laneletl.pose.orientation.z = 0.0;

    laneletr.header.frame_id = "map";
    laneletr.header.stamp = ros::Time::now();
    laneletr.ns = "neighbor_laneletr";
    laneletr.action = visualization_msgs::Marker::ADD;
    laneletr.id = 1;
    laneletr.type = visualization_msgs::Marker::LINE_STRIP;
    laneletr.scale.x = 1.0;
    laneletr.color.r = 1.0;
    laneletr.color.g = 0.0;
    laneletr.color.b = 0.0;
    laneletr.color.a = 0.8;
    laneletr.pose.orientation.w = 1.0;
    laneletr.pose.orientation.x = 0.0;
    laneletr.pose.orientation.y = 0.0;
    laneletr.pose.orientation.z = 0.0;

    laneletl.points.insert(laneletl.points.end(), test_msg.response.Left_path.points.begin(), 
                                                  test_msg.response.Left_path.points.end());
    laneletsarray.markers.push_back(laneletl);
    laneletr.points.insert(laneletr.points.end(), test_msg.response.Right_path.points.begin(), 
                                                  test_msg.response.Right_path.points.end());
    laneletsarray.markers.push_back(laneletr);
    pub.publish(laneletsarray);

    ROS_INFO("left size: %ld, right size: %ld", test_msg.response.Left_path.points.size(), 
                                                test_msg.response.Right_path.points.size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_getneighborlanelet");
    ros::NodeHandle nh("~");
    client = nh.serviceClient<osmmap::GetNeighborLanelet>("/navagation_node/getneighborlanelet_service");
    pub = nh.advertise<visualization_msgs::MarkerArray>("/test_neighborlanelet", 1);
    ros::Subscriber publishpoint_sub = nh.subscribe("/initialpose", 1, callback);

    ros::spin();
    return 0;
}

