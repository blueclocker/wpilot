#include "optimizer/optimizer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_optimizer_node");
    ros::NodeHandle nh;
    ros::spin();
    
    return 0;
}