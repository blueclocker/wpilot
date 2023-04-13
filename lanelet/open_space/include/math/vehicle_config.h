/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-17 14:55:11
 * @LastEditTime: 2023-02-23 17:26:43
 * @LastEditors: wpbit
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/open_space/include/math/vehicle_config.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef VEHICLE_CONFIG_H_
#define VEHICLE_CONFIG_H_

namespace apollo {
namespace common {
struct VehicleParam {
//   VehicleBrand brand;
  // Car center point is car reference point, i.e., center of rear axle.
//   VehicleID vehicle_id;
  double front_edge_to_center = 10.95;//3.89;
  double back_edge_to_center = 6.205;//1.043;
  double left_edge_to_center = 1.54;//1.055;
  double right_edge_to_center = 1.54;//1.055;

  double length = 17.155;//4.933;
  double width = 3.08;//2.11;
  double height = 2.5;//1.48;

  double min_turn_radius = 15.0;//5.05386147161;
  double max_acceleration = 3.2;//2.0;
  double max_deceleration = -6.0;

  // The following items are used to compute trajectory constraints in
  // planning/control/canbus,
  // vehicle max steer angle
  double max_steer_angle = 8.20304748437;
  // vehicle max steer rate; how fast can the steering wheel turn.
  double max_steer_angle_rate = 6.98131700798;
  // vehicle min steer rate;
  double min_steer_angle_rate = 0;
  // ratio between the turn of steering wheel and the turn of wheels
  double steer_ratio = 16;
  // the distance between the front and back wheels
  double wheel_base = 2.108;//2.8448;
  // Tire effective rolling radius (vertical distance between the wheel center
  // and the ground).
  double wheel_rolling_radius = 0.5;//0.335;

  // minimum differentiable vehicle speed, in m/s
  float max_abs_speed_when_stopped = 0.2;

  // minimum value get from chassis.brake, in percentage
  double brake_deadzone = 14.5;
  // minimum value get from chassis.throttle, in percentage
  double throttle_deadzone = 15.7;

  // vehicle latency parameters
//   LatencyParam steering_latency_param;
//   LatencyParam throttle_latency_param;
//   LatencyParam brake_latency_param;
};


}
}
#endif