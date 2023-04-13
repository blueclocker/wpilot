/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "smoother/discrete_points_reference_line_smoother.h"

#include <algorithm>

// #include "modules/common/time/time.h"
// #include "modules/common/util/util.h"
#include <chrono>

#include "math/planning_gflags.h"
#include "math/discrete_points_math.h"
#include "math/cos_theta_smoother.h"
#include "math/fem_pos_deviation_smoother.h"

namespace apollo {
namespace planning {

DiscretePointsReferenceLineSmoother::DiscretePointsReferenceLineSmoother(
    const ReferenceLineSmootherConfig& config)
    : ReferenceLineSmoother(config) {}

bool DiscretePointsReferenceLineSmoother::Smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line) {
  const auto start_timestamp = std::chrono::system_clock::now();

  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<double> anchorpoints_lateralbound;

// anchor_points_在别处设置的，根据车宽、道路是否是curb等对原始车道进行收缩，anchor_points就在
// 收缩后的道路中心，lateral_bound就是收缩后道路一半宽度。
  for (const auto& anchor_point : anchor_points_) {
    raw_point2d.emplace_back(anchor_point.path_point.x(),
                             anchor_point.path_point.y());
    anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
  }

  // fix front and back points to avoid end states deviate from the center of
  // road
  anchorpoints_lateralbound.front() = 0.0;
  anchorpoints_lateralbound.back() = 0.0;

  // 点的坐标与第一个点的坐标相减。
  NormalizePoints(&raw_point2d);

  const auto solver_start_timestamp = std::chrono::system_clock::now();

  bool status = false;

  const auto& smoothing_method = config_.discrete_points.smoothing_method;
  std::vector<std::pair<double, double>> smoothed_point2d;
  switch (smoothing_method) {
    case SmoothingMethod::COS_THETA_SMOOTHING:
      status = CosThetaSmooth(raw_point2d, anchorpoints_lateralbound,
                              &smoothed_point2d);
      break;
    case SmoothingMethod::FEM_POS_DEVIATION_SMOOTHING:
      status = FemPosSmooth(raw_point2d, anchorpoints_lateralbound,
                            &smoothed_point2d);
      break;
    default:
      std::cout << "Smoother type not defined" << std::endl;
      return false;
  }

  if (!status) {
    std::cout << "discrete_points reference line smoother fails" << std::endl;
    return false;
  }

  const auto solver_end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> solver_diff =
      solver_end_timestamp - solver_start_timestamp;
  std::cout << "discrete_points reference line smoother solver time: "
         << solver_diff.count() * 1000.0 << " ms." << std::endl;

  DeNormalizePoints(&smoothed_point2d);

  std::vector<ReferencePoint> ref_points;
  GenerateRefPointProfile(raw_reference_line, smoothed_point2d, &ref_points);

  ReferencePoint::RemoveDuplicates(&ref_points);

  if (ref_points.size() < 2) {
    std::cout << "Fail to generate smoothed reference line." << std::endl;
    return false;
  }

  *smoothed_reference_line = ReferenceLine(ref_points);

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  std::cout << "discrete_points reference line smoother totoal time: "
         << diff.count() * 1000.0 << " ms." << std::endl;

  return true;
}

bool DiscretePointsReferenceLineSmoother::CosThetaSmooth(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  const auto& cos_theta_config =
      config_.discrete_points.cos_theta_smoothing;

  CosThetaSmoother smoother(cos_theta_config);

  // box contraints on pos are used in cos theta smoother, thus shrink the
  // bounds by 1.0 / sqrt(2.0)
  std::vector<double> box_bounds = bounds;
  const double box_ratio = 1.0 / std::sqrt(2.0);
  for (auto& bound : box_bounds) {
    bound *= box_ratio;
  }

  std::vector<double> opt_x;
  std::vector<double> opt_y;
  bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

  if (!status) {
    std::cout << "Costheta reference line smoothing failed" << std::endl;
    return false;
  }

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    std::cout << "Return by Costheta smoother is wrong. Size smaller than 2 " << std::endl;
    return false;
  }

  CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

  size_t point_size = opt_x.size();
  for (size_t i = 0; i < point_size; ++i) {
    ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}

bool DiscretePointsReferenceLineSmoother::FemPosSmooth(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  const auto& fem_pos_config =
      config_.discrete_points.fem_pos_deviation_smoothing;

  FemPosDeviationSmoother smoother(fem_pos_config);

  // box contraints on pos are used in fem pos smoother, thus shrink the
  // bounds by 1.0 / sqrt(2.0)
  std::vector<double> box_bounds = bounds;
  const double box_ratio = 1.0 / std::sqrt(2.0);
  for (auto& bound : box_bounds) {
    bound *= box_ratio;
  }

  std::vector<double> opt_x;
  std::vector<double> opt_y;
  bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

  if (!status) {
    std::cout << "Fem Pos reference line smoothing failed" << std::endl;
    return false;
  }

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    std::cout << "Return by fem pos smoother is wrong. Size smaller than 2 " << std::endl;
    return false;
  }

  CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

  size_t point_size = opt_x.size();
  for (size_t i = 0; i < point_size; ++i) {
    ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}

void DiscretePointsReferenceLineSmoother::SetAnchorPoints(
    const std::vector<AnchorPoint>& anchor_points) {
  CHECK_GT(anchor_points.size(), 1);
  anchor_points_ = anchor_points;
}

void DiscretePointsReferenceLineSmoother::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}

void DiscretePointsReferenceLineSmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}

bool DiscretePointsReferenceLineSmoother::GenerateRefPointProfile(
    const ReferenceLine& raw_reference_line,
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<ReferencePoint>* reference_points) {
  // Compute path profile
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (!DiscretePointsMath::ComputePathProfile(
          xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }

  // Load into ReferencePoints
  size_t points_size = xy_points.size();
  for (size_t i = 0; i < points_size; ++i) {
    common::SLPoint ref_sl_point;
    if (!raw_reference_line.XYToSL({xy_points[i].first, xy_points[i].second},
                                   &ref_sl_point)) {
      return false;
    }
    const double kEpsilon = 1e-6;
    if (ref_sl_point.s() < -kEpsilon ||
        ref_sl_point.s() > raw_reference_line.Length()) {
      continue;
    }
    ref_sl_point.set_s(std::max(ref_sl_point.s(), 0.0));
    ReferencePoint rlp = raw_reference_line.GetReferencePoint(ref_sl_point.s());
    auto new_lane_waypoints = rlp.lane_waypoints();
    for (auto& lane_waypoint : new_lane_waypoints) {
      lane_waypoint.l = ref_sl_point.l();
    }
    reference_points->emplace_back(ReferencePoint(
        hdmap::MapPathPoint(
            common::math::Vec2d(xy_points[i].first, xy_points[i].second),
            headings[i], new_lane_waypoints),
        kappas[i], dkappas[i]));
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
