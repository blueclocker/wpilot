#pragma once


#include "data_struct.h"
#include <queue>
#include <unordered_map>


namespace optimizer
{
struct PlannerResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> steer;
  std::vector<double> accumulated_s;

  void clear()
  {
    x.clear();
    y.clear();
    phi.clear();
    v.clear();
    steer.clear();
    accumulated_s.clear();
  }
};

class UnionPlanner
{
private:
    PlannerOpenSpaceConfig planner_open_space_config_;
    VehicleParam vehicle_param_;
    // common::VehicleParam vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
    size_t next_node_num_ = 0;
    double max_steer_angle_ = 0.0;
    double step_size_ = 0.0;
    double xy_grid_resolution_ = 0.0;
    double delta_t_ = 0.0;
    double traj_steer_penalty_ = 0.0;
    double traj_steer_change_penalty_ = 0.0;
    double traj_v_penalty_ = 0.0;
    double traj_v_change_penalty_ = 0.0;
    double traj_phi_penalty_ = 0.0;
    double traj_l_penalty_ = 0.0;
    double traj_s_penalty_ = 0.0;
    double heu_remain_distance_penalty_ = 0.0;
    double heu_l_diff_penalty_ = 0.0;
    std::vector<double> XYbounds_;
    std::shared_ptr<Node3d> start_node_;
    std::shared_ptr<Node3d> end_node_;
    std::shared_ptr<Node3d> final_node_;
    std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec_;
    std::vector<Vec2d> globalpath_;

    struct cmp {
        bool operator()(const std::pair<std::string, double>& left,
                        const std::pair<std::string, double>& right) const {
        return left.second >= right.second;
        }
    };
    std::priority_queue<std::pair<std::string, double>,
                        std::vector<std::pair<std::string, double>>, cmp>
        open_pq_;
    std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;
    std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;
    // std::unique_ptr<ReedShepp> reed_shepp_generator_;
    // std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;

    // check collision and validity
    bool ValidityCheck(std::shared_ptr<Node3d> node);
    std::shared_ptr<Node3d> Next_node_generator(
        std::shared_ptr<Node3d> current_node, size_t next_node_index, double traveled_distance);
    void CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                            std::shared_ptr<Node3d> next_node);
    double TrajCost(std::shared_ptr<Node3d> current_node,
                    std::shared_ptr<Node3d> next_node);
    double HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node);
    bool GetResult(PlannerResult* result);
    // bool GetTemporalProfile(PlannerResult* result);
    // bool GenerateSpeedAcceleration(PlannerResult* result);
    // bool GenerateSCurveSpeedAcceleration(PlannerResult* result);
public:
    explicit UnionPlanner(const PlannerOpenSpaceConfig& open_space_conf);
    virtual ~UnionPlanner() = default;
    bool Plan(double sx, double sy, double sphi, double sv, double ex, double ey,
            double ephi, const std::vector<double>& XYbounds,
            const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
            const std::vector<Vec2d>& globalpath_vec,
            PlannerResult* result);
};


}// namespace optimizer