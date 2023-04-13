/*
 * @Author: wpbit
 * @Date: 2023-02-14 21:41:03
 * @LastEditors: wpbit
 * @LastEditTime: 2023-02-14 22:57:00
 * @Description: 
 */
#include "navagation/navagation_optimizer.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/MarkerArray.h"

ros::ServiceClient client;
ros::Publisher pub;

//输入当前车在地图坐标系的xy坐标和航向角(实际未用)
//返回的markers与自车道同向的所有车道与当前车道的下一车道及其同向所有车道，点集顺序：从左侧到右侧顺时针依次连接
//(TODO)暂不支持纵向扩展，只支持当前同向车道取到尽头
void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_INFO("get current point!");
    osmmap::GetSameDirectionLanelet test_msg;
    test_msg.request.Sx = msg->pose.pose.position.x;
    test_msg.request.Sy = msg->pose.pose.position.y;
    test_msg.request.Syaw = tf::getYaw(msg->pose.pose.orientation);
    bool f = client.call(test_msg);

    //visualization
    visualization_msgs::MarkerArray laneletsarray;
    for(int i = 0; i < test_msg.response.lanelets.size(); ++i)
    {
        test_msg.response.lanelets[i].header.frame_id = "map";
        test_msg.response.lanelets[i].header.stamp = ros::Time::now();
        test_msg.response.lanelets[i].ns = "neighbor_laneletl";
        test_msg.response.lanelets[i].action = visualization_msgs::Marker::ADD;
        test_msg.response.lanelets[i].id = i;
        test_msg.response.lanelets[i].type = visualization_msgs::Marker::LINE_STRIP;
        test_msg.response.lanelets[i].scale.x = 0.2;
        test_msg.response.lanelets[i].color.r = 1.0;
        test_msg.response.lanelets[i].color.g = 0.0;
        test_msg.response.lanelets[i].color.b = 0.0;
        test_msg.response.lanelets[i].color.a = 0.8;
        test_msg.response.lanelets[i].pose.orientation.w = 1.0;
        test_msg.response.lanelets[i].pose.orientation.x = 0.0;
        test_msg.response.lanelets[i].pose.orientation.y = 0.0;
        test_msg.response.lanelets[i].pose.orientation.z = 0.0;
        laneletsarray.markers.push_back(test_msg.response.lanelets[i]);
    }
    pub.publish(laneletsarray);
    ROS_INFO("lanelet size: %ld", test_msg.response.lanelets.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_getsamedirectionlanelet");
    ros::NodeHandle nh;
    client = nh.serviceClient<osmmap::GetSameDirectionLanelet>("/navagation_node/getsamedirectionlanelet_service");
    pub = nh.advertise<visualization_msgs::MarkerArray>("/test_samedirectionlanelet", 1);
    ros::Subscriber publishpoint_sub = nh.subscribe("/initialpose", 1, callback);

    ros::spin();
    return 0;
}
