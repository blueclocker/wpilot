/*
 * @Author: wpbit
 * @Date: 2023-04-04 10:03:26
 * @LastEditors: wpbit
 * @LastEditTime: 2023-04-04 10:06:11
 * @Description: 
 */
#include "rrtstar/navagation_rrtstar.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrtstar");
    ros::NodeHandle n("~");
    RRT::NavagationRRTstar navagationrrt(n);
    navagationrrt.run();
    return 0;
}
