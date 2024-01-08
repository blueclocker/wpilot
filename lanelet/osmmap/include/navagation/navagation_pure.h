/*
 * @Author: wpbit
 * @Date: 2023-02-14 13:14:19
 * @LastEditors: wpbit
 * @LastEditTime: 2023-02-16 15:46:32
 * @Description: 
 */
#ifndef NAVAGATION_PURE_H_
#define NAVAGATION_PURE_H_

#include "navagation.h"
#include <nav_msgs/Odometry.h>

namespace navagation
{
class NavagationPure : public NavagationBase
{
private:
    void PureCallback(const nav_msgs::Odometry::ConstPtr &msg);
    ros::Publisher car_model_pub_;
public:
    NavagationPure(ros::NodeHandle &n);
    virtual ~NavagationPure() override = default;
    virtual void Process() override;
};

}// namespace navagation

#endif