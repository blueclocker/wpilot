/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-16 15:25:18
 * @LastEditTime: 2022-09-16 15:28:02
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/rrtstar/src/rrt_test.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "rrtstar/rrt_flow.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_demo");
    ros::NodeHandle nh("~");
    RRT::RRTflow rrtflow(nh);

    while(nh.ok())
    {
        rrtflow.run();
        
        ros::spinOnce();
        ros::Rate(10).sleep();
    }
    return 0;
}