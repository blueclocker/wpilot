#include "optimizer/data_struct.h"
#include "absl/strings/str_cat.h"

namespace optimizer
{
Node3d::Node3d(double x, double y, double phi, const double v, const double steering) {
  x_ = x;
  y_ = y;
  phi_ = phi;
  v_ = v;
  steering_ = steering;
}

Node3d::Node3d(double x, double y, double phi, const double v, const double steering,
               const std::vector<double>& XYbounds,
               const PlannerOpenSpaceConfig& open_space_conf) {
  CHECK_EQ(XYbounds.size(), 4)
      << "XYbounds size is not 4, but" << XYbounds.size();

  x_ = x;
  y_ = y;
  phi_ = phi;
  v_ = v;
  steering_ = steering;

  x_grid_ = static_cast<int>(
      (x_ - XYbounds[0]) /
      open_space_conf.warm_start_config.xy_grid_resolution);
  y_grid_ = static_cast<int>(
      (y_ - XYbounds[2]) /
      open_space_conf.warm_start_config.xy_grid_resolution);
  phi_grid_ = static_cast<int>(
      (phi_ - (-M_PI)) /
      open_space_conf.warm_start_config.phi_grid_resolution);

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
}

Box2d Node3d::GetBoundingBox(const VehicleParam& vehicle_param_,
                             const double x, const double y, const double phi) {
  double ego_length = vehicle_param_.length;
  double ego_width = vehicle_param_.width;
  double shift_distance =
      ego_length / 2.0 - vehicle_param_.back_edge_to_center;
  Box2d ego_box(
      {x + shift_distance * std::cos(phi), y + shift_distance * std::sin(phi)},
      phi, ego_length, ego_width);
  return ego_box;
}

bool Node3d::operator==(const Node3d& right) const {
  return right.GetIndex() == index_;
}

std::string Node3d::ComputeStringIndex(int x_grid, int y_grid, int phi_grid) {
  return absl::StrCat(x_grid, "_", y_grid, "_", phi_grid);
}

void Node3d::SetSL(const std::vector<Vec2d> &globalpath)
{
  if(globalpath.empty()) return;

  double dis = std::numeric_limits<double>::max();
  Vec2d current_node = Vec2d(x_, y_);
  int count = 0;
  for(const auto& node : globalpath)
  {
    double current_dis = current_node.DistanceTo(node);
    if(current_dis < dis){
      dis = current_dis;
      count++;
    }else{
      break;
    }
  }
  l_ = dis;
  s_ = count * 0.2;
  std::cout << "node s: " << s_ << ", l: " << l_ << std::endl;
}


}//namespace optimizer