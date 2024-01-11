#pragma once

#include <iostream>
#include <memory>
#include <deque>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <glog/logging.h>
#include "box2d.h"
#include "math_utils.h"
#include "polygon2d.h"

namespace optimizer
{
struct VehicleParam {
//   VehicleBrand brand;
// Car center point is car reference point, i.e., center of rear axle.
//   VehicleID vehicle_id;
  double front_edge_to_center = 3.89;
  double back_edge_to_center = 1.043;
  double left_edge_to_center = 1.055;
  double right_edge_to_center = 1.055;

  double length = 4.933;
  double width = 2.11;
  double height = 1.48;

  double min_turn_radius = 5.05386147161;
  double max_acceleration = 2.0;
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
  double wheel_base = 2.8448;
  // Tire effective rolling radius (vertical distance between the wheel center
  // and the ground).
  double wheel_rolling_radius = 0.335;

  // minimum differentiable vehicle speed, in m/s
  float max_abs_speed_when_stopped = 0.2;

  // minimum value get from chassis.brake, in percentage
  double brake_deadzone = 14.5;
  // minimum value get from chassis.throttle, in percentage
  double throttle_deadzone = 15.7;
};

struct WarmStartConfig {
  // Hybrid a star for warm start
  double xy_grid_resolution = 0.2;
  double phi_grid_resolution = 0.1;//0.05
  u_int64_t next_node_num = 7;//10
  double step_size = 0.5;
  // double traj_forward_penalty = 0;//0
  // double traj_back_penalty = 10.0;//0
  // double traj_gear_switch_penalty = 100.0;//10
  double traj_steer_penalty = 30.0;//100
  double traj_steer_change_penalty = 20.0;//10
  double traj_v_penalty = 30.0;
  double traj_v_change_penalty = 20.0;
  double traj_l_penalty = 30.0;
  double traj_s_penalty = 10.0;

  double heu_remain_distance_penalty = 10.0;
  double heu_l_diff_penalty = 20.0;
  // Grid a star for heuristic
  double grid_a_star_xy_resolution = 0.1;
  double node_radius = 0.5;
};

struct PlannerOpenSpaceConfig
{
  WarmStartConfig warm_start_config;
  float delta_t = 1.0;//1.0
  double is_near_destination_threshold = 0.001;
  bool enable_check_parallel_trajectory = false;
  bool enable_linear_interpolation = false;
  double is_near_destination_theta_threshold = 0.05;
};

class Node3d {
 public:
  Node3d(const double x, const double y, const double phi, const double v, const double steering);
  Node3d(const double x, const double y, const double phi, const double v, const double steering,
         const std::vector<double>& XYbounds,
         const PlannerOpenSpaceConfig& open_space_conf);
  virtual ~Node3d() = default;
  static Box2d GetBoundingBox(
      const VehicleParam& vehicle_param_, const double x,
      const double y, const double phi);
  double GetCost() const { return traj_cost_ + heuristic_cost_; }
  double GetTrajCost() const { return traj_cost_; }
  double GetHeuCost() const { return heuristic_cost_; }
  int GetGridX() const { return x_grid_; }
  int GetGridY() const { return y_grid_; }
  int GetGridPhi() const { return phi_grid_; }
  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetPhi() const { return phi_; }
  double GetV() const {return v_;}
  double GetS() const {return s_;}
  double GetL() const {return l_;}
  bool operator==(const Node3d& right) const;
  const std::string& GetIndex() const { return index_; }
  // size_t GetStepSize() const { return step_size_; }
  // bool GetDirec() const { return direction_; }
  double GetSteer() const { return steering_; }
  std::shared_ptr<Node3d> GetPreNode() const { return pre_node_; }
  void SetPre(std::shared_ptr<Node3d> pre_node) { pre_node_ = pre_node; }
  // void SetDirec(bool direction) { direction_ = direction; }
  void SetTrajCost(double cost) { traj_cost_ = cost; }
  void SetHeuCost(double cost) { heuristic_cost_ = cost; }
  // void SetSteer(double steering) { steering_ = steering; }
  void SetSL(const std::vector<Vec2d> &globalpath);

 private:
  static std::string ComputeStringIndex(int x_grid, int y_grid, int phi_grid);

 private:
  double x_ = 0.0;//m
  double y_ = 0.0;//m
  double phi_ = 0.0;//rad
  double v_ = 0.0;//m/s
  double steering_ = 0.0;//rad
  double s_ = 0.0;
  double l_ = 0.0;

  // size_t step_size_ = 1;
  int x_grid_ = 0;
  int y_grid_ = 0;
  int phi_grid_ = 0;
  std::string index_;

  double traj_cost_ = 0.0;
  double heuristic_cost_ = 0.0;
  double cost_ = 0.0;
  std::shared_ptr<Node3d> pre_node_ = nullptr;
  // true for moving forward and false for moving backward
  // bool direction_ = true;
};


}//namespace optimizer
