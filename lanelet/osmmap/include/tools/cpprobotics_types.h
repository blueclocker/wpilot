/*
 * @Author: wpbit
 * @Date: 2023-01-06 17:20:44
 * @LastEditors: wpbit
 * @LastEditTime: 2023-03-19 15:56:25
 * @Description: 
 */

#ifndef CPPROBOTICS_TYPES_H_
#define CPPROBOTICS_TYPES_H_

#include<iterator>
#include<vector>
#include<array>
#include<string>
#include<iostream>

namespace cpprobotics{

using Vec_f=std::vector<double>;
using Poi_f=std::array<double, 2>;
using Vec_Poi=std::vector<Poi_f>;

};

#endif
