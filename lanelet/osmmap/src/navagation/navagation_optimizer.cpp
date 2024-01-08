/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-06 21:32:30
 * @LastEditTime: 2023-02-14 13:29:35
 * @LastEditors: wpbit
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/navagation/navagation_optimizer.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "navagation/navagation_optimizer.h"


namespace navagation
{
NavagationOptimizer::NavagationOptimizer(const std::string &mode, ros::NodeHandle &n) : mode_(mode), n_(n)
{
    if(mode == "GNSS")
    {
        navagation_ptr_ = new NavagationGnss(n);
    }else if(mode == "PCD"){
        navagation_ptr_ = new NavagationPcd(n);
    }else if(mode == "SIM"){
        navagation_ptr_ = new NavagationSim(n);
    }else if(mode == "PURE"){
        navagation_ptr_ = new NavagationPure(n);
    }else{
        ROS_ERROR("error navagation mode, please check!");
        return;
    }

    //start navagation
    navagation_ptr_->Process();
}

NavagationOptimizer::~NavagationOptimizer()
{
    // CHECK_NOTNULL(navagation_ptr_);
    if(!navagation_ptr_) delete navagation_ptr_;
}



}//namespace navagation