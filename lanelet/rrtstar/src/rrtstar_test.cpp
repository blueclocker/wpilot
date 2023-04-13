/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-16 20:40:17
 * @LastEditTime: 2022-09-16 22:19:28
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/rrtstar/src/rrtstar_test.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "rrtstar/rrtstar_flow.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrtstar_demo");
    ros::NodeHandle nh("~");
    RRT::RRTStarflow rrtstarflow(nh);
    ros::AsyncSpinner s(3);
    s.start();

    while(nh.ok())
    {
        rrtstarflow.run();

        ros::spinOnce();
        ros::Rate(10).sleep();
    }
    return 0;
}
