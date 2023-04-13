/*
 * @Author: wpbit
 * @Date: 2023-03-18 18:55:59
 * @LastEditors: wpbit
 * @LastEditTime: 2023-03-18 18:57:37
 * @Description: 
 */
#include "plan/lattice_plan.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lattice_node");
    ros::NodeHandle nh("~");
    plan::LatticePlan planner(nh);

    return 0;
}