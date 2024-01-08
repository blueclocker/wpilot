#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    static tf::TransformBroadcaster broadcaster_;
    tf::Transform baselink2map_;
    tf::Quaternion tf_q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation,tf_q);
    baselink2map_.setRotation(tf_q);
    baselink2map_.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    //map->rslidar->base_link
    broadcaster_.sendTransform(tf::StampedTransform(baselink2map_, msg->header.stamp, "map", "base_link"));
    std::cout << "atnowpoint update" << std::endl;
    std::cout << "now x: " << msg->pose.pose.position.x << ", now y: " << msg->pose.pose.position.y << ", now z: " << msg->pose.pose.position.z << std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_pub");
    ros::NodeHandle nh("~");
    ros::Subscriber sub = nh.subscribe("/carla/ego_vehicle/odometry", 1, callback);

    ros::spin();
    return 0;
}