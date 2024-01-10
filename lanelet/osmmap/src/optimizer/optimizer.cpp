#include "optimizer/optimizer.h"

namespace optimizer
{
UnionPlanner::UnionPlanner(const PlannerOpenSpaceConfig& open_space_conf)
{
    // planner_open_space_config_.CopyFrom(open_space_conf);
    planner_open_space_config_ = open_space_conf;
    // reed_shepp_generator_ =
        // std::make_unique<ReedShepp>(vehicle_param_, planner_open_space_config_);
    // reed_shepp_generator_ = std::unique_ptr<ReedShepp>(new ReedShepp(vehicle_param_, planner_open_space_config_));
    // grid_a_star_heuristic_generator_ =
        // std::make_unique<GridSearch>(planner_open_space_config_);
    // grid_a_star_heuristic_generator_ = 
    //     std::unique_ptr<GridSearch>(new GridSearch(planner_open_space_config_));
    next_node_num_ =
        planner_open_space_config_.warm_start_config.next_node_num;
    max_steer_angle_ =
        vehicle_param_.max_steer_angle / vehicle_param_.steer_ratio;
    step_size_ = planner_open_space_config_.warm_start_config.step_size;
    xy_grid_resolution_ =
        planner_open_space_config_.warm_start_config.xy_grid_resolution;
    delta_t_ = planner_open_space_config_.delta_t;
    traj_forward_penalty_ =
        planner_open_space_config_.warm_start_config.traj_forward_penalty;
    traj_back_penalty_ =
        planner_open_space_config_.warm_start_config.traj_back_penalty;
    traj_gear_switch_penalty_ =
        planner_open_space_config_.warm_start_config.traj_gear_switch_penalty;
    traj_steer_penalty_ =
        planner_open_space_config_.warm_start_config.traj_steer_penalty;
    traj_steer_change_penalty_ = 
        planner_open_space_config_.warm_start_config.traj_steer_change_penalty;
    traj_v_change_penalty_ = 
        planner_open_space_config_.warm_start_config.traj_v_change_penalty;
}

bool UnionPlanner::Plan(double sx, double sy, double sphi, double sv, double ex, double ey,
                        double ephi, const std::vector<double>& XYbounds,
                        const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
                        PlannerResult* result)
{
    // clear containers
    open_set_.clear();
    close_set_.clear();
    open_pq_ = decltype(open_pq_)();
    final_node_ = nullptr;
    const auto start_timestamp = std::chrono::system_clock::now();

    std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec;
    for (const auto& obstacle_vertices : obstacles_vertices_vec) {
        size_t vertices_num = obstacle_vertices.size();
        std::vector<LineSegment2d> obstacle_linesegments;
        for (size_t i = 0; i < vertices_num - 1; ++i) {
            LineSegment2d line_segment = LineSegment2d(obstacle_vertices[i], obstacle_vertices[i + 1]);
            obstacle_linesegments.emplace_back(line_segment);
        }
        obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
    }
    obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

    // load XYbounds
    XYbounds_ = XYbounds;
    // load nodes and obstacles
    start_node_.reset(
        new Node3d({sx}, {sy}, {sphi}, {sv}, {0}, XYbounds_, planner_open_space_config_));
    end_node_.reset(
        new Node3d({ex}, {ey}, {ephi}, {0}, {0}, XYbounds_, planner_open_space_config_));
    if (!ValidityCheck(start_node_)) {
        std::cout << "start_node in collision with obstacles" << std::endl;
        return false;
    }
    if (!ValidityCheck(end_node_)) {
        std::cout << "end_node in collision with obstacles" << std::endl;
        return false;
    }
    // double map_time = Clock::NowInSeconds();
    // grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_,
    //                                                 obstacles_linesegments_vec_);
    // std::cout << "map time " << Clock::NowInSeconds() - map_time << std::endl;
    // const auto map_timestamp = std::chrono::system_clock::now();
    // std::chrono::duration<double> mapdiff = map_timestamp - start_timestamp;
    // std::cout << "map total time is " << mapdiff.count() * 1000.0 << " ms." << std::endl;
    // load open set, pq
    open_set_.emplace(start_node_->GetIndex(), start_node_);
    open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

    // const auto start_timestamp = std::chrono::system_clock::now();
    // Hybrid A* begins
    size_t explored_node_num = 0;
    // double astar_start_time = Clock::NowInSeconds();
    // double heuristic_time = 0.0;
    // double rs_time = 0.0;
    while (!open_pq_.empty()) {
        // take out the lowest cost neighboring node
        const std::string current_id = open_pq_.top().first;
        open_pq_.pop();
        std::shared_ptr<Node3d> current_node = open_set_[current_id];
        // check if an analystic curve could be connected from current
        // configuration to the end configuration without collision. if so, search
        // ends.
        // const double rs_start_time = Clock::NowInSeconds();
        // 搜索成功退出点
        if (std::sqrt(std::pow((current_node->GetX() - end_node_->GetX()), 2) + 
                    std::pow((current_node->GetY() - end_node_->GetY()), 2)) < 1.0) {
            final_node_ = current_node;
            break;
        }
        // const double rs_end_time = Clock::NowInSeconds();
        // rs_time += rs_end_time - rs_start_time;
        close_set_.emplace(current_node->GetIndex(), current_node);

        double start_travel_distance = std::max(sv * 0.1 - 2.0, 0.0);
        double end_travle_distance = std::min(sv * 0.1 + 2.0, 5.0);;
        for(double travle_dis = start_travel_distance; 
            travle_dis <= end_travle_distance; travle_dis += 0.5)
        {
            for (size_t i = 0; i < next_node_num_; ++i) 
            {
                // std::cout << "generate node travel distance: " << travle_dis << ", steer: " << i << std::endl;
                std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i, travle_dis);
                // boundary check failure handle
                if (next_node == nullptr) {
                    continue;
                }
                // check if the node is already in the close set
                if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
                    continue;
                }
                // collision check
                if (!ValidityCheck(next_node)) {
                    continue;
                }
                if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
                    explored_node_num++;
                    // const double start_time = Clock::NowInSeconds();
                    CalculateNodeCost(current_node, next_node);
                    // const double end_time = Clock::NowInSeconds();
                    // heuristic_time += end_time - start_time;
                    open_set_.emplace(next_node->GetIndex(), next_node);
                    open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
                }
            }
        }

        //Hybrid A star > 500 ms搜索失败
        // const auto now_timestamp = std::chrono::system_clock::now();
        // std::chrono::duration<double> diff = now_timestamp - start_timestamp;
        // if(diff.count() * 1000.0 > 500.0) 
        // {
        //   std::cout << "Hybrid A has run out of 50 ms" << std::endl;
        //   return false;
        // }
    }
    if (final_node_ == nullptr) {
        std::cout << "Hybrid A searching return null ptr(open_set ran out)" << std::endl;
        return false;
    }
    if (!GetResult(result)) {
        std::cout << "GetResult failed" << std::endl;
        return false;
    }
    std::cout << "explored node num is " << explored_node_num << std::endl;
    // std::cout << "heuristic time is " << heuristic_time << std::endl;
    // std::cout << "reed shepp time is " << rs_time << std::endl;
    // std::cout << "hybrid astar total time is "
    //        << Clock::NowInSeconds() - astar_start_time << std::endl;
    const auto end_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_timestamp - start_timestamp;
    std::cout << "hybrid astar total time is " << diff.count() * 1000.0 << " ms." << std::endl;
    return true;
}

bool UnionPlanner::ValidityCheck(std::shared_ptr<Node3d> node)
{
    CHECK_NOTNULL(node);
    // CHECK_GT(node->GetStepSize(), 0);

    if (obstacles_linesegments_vec_.empty()) {
        return true;
    }

    // size_t node_step_size = node->GetStepSize();
    const auto& traversed_x = node->GetX();
    const auto& traversed_y = node->GetY();
    const auto& traversed_phi = node->GetPhi();

    // The first {x, y, phi} is collision free unless they are start and end
    // configuration of search problem
    // size_t check_start_index = 0;
    // if (node_step_size == 1) {
    //     check_start_index = 0;
    // } else {
    //     check_start_index = 1;
    // }

    // for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x > XYbounds_[1] || traversed_x < XYbounds_[0] ||
        traversed_y > XYbounds_[3] || traversed_y < XYbounds_[2]) {
        return false;
    }
    Box2d bounding_box = Node3d::GetBoundingBox(
        vehicle_param_, traversed_x, traversed_y, traversed_phi);
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
        for (const LineSegment2d& linesegment : obstacle_linesegments) {
            if (bounding_box.HasOverlap(linesegment)) {
                // std::cout << "collision start at x: " << linesegment.start().x() << std::endl;
                // std::cout << "collision start at y: " << linesegment.start().y() << std::endl;
                // std::cout << "collision end at x: " << linesegment.end().x() << std::endl;
                // std::cout << "collision end at y: " << linesegment.end().y() << std::endl;
                return false;
            }
        }
    }
    // }
    return true;
}

std::shared_ptr<Node3d> UnionPlanner::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index, double traveled_distance)
{
    double steering = 0.0;
    if (next_node_index < static_cast<double>(next_node_num_) / 2) {
        steering =
            -max_steer_angle_ +
            (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
                static_cast<double>(next_node_index);
    } else {
        size_t index = next_node_index - next_node_num_ / 2;
        steering =
            -max_steer_angle_ +
            (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
                static_cast<double>(index);
    }
    // take above motion primitive to generate a curve driving the car to a
    // different grid
    // double arc = std::sqrt(2) * xy_grid_resolution_;
    // std::vector<double> intermediate_x;
    // std::vector<double> intermediate_y;
    // std::vector<double> intermediate_phi;
    double last_x = current_node->GetX();
    double last_y = current_node->GetY();
    double last_phi = current_node->GetPhi();
    // intermediate_x.push_back(last_x);
    // intermediate_y.push_back(last_y);
    // intermediate_phi.push_back(last_phi);
    // for (size_t i = 0; i < arc / step_size_; ++i) {
    //     const double next_x = last_x + traveled_distance * std::cos(last_phi);
    //     const double next_y = last_y + traveled_distance * std::sin(last_phi);
    //     const double next_phi = NormalizeAngle(
    //         last_phi +
    //         traveled_distance / vehicle_param_.wheel_base * std::tan(steering));
    //     intermediate_x.push_back(next_x);
    //     intermediate_y.push_back(next_y);
    //     intermediate_phi.push_back(next_phi);
    //     last_x = next_x;
    //     last_y = next_y;
    //     last_phi = next_phi;
    // }
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    const double next_phi = NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base * std::tan(steering));
    const double next_v = traveled_distance / 0.1; // 每个采样点之间间隔0.1s
    // check if the vehicle runs outside of XY boundary
    if (next_x > XYbounds_[1] ||
        next_x < XYbounds_[0] ||
        next_y > XYbounds_[3] ||
        next_y < XYbounds_[2]) {
        return nullptr;
    }
    std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
        new Node3d(next_x, next_y, next_phi, next_v, steering, XYbounds_,
                    planner_open_space_config_));
    next_node->SetPre(current_node);
    // next_node->SetDirec(traveled_distance > 0.0);
    // next_node->SetSteer(steering);
    return next_node;
}

void UnionPlanner::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                     std::shared_ptr<Node3d> next_node)
{
    next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));
    // evaluate heuristic cost
    double optimal_path_cost = 0.0;
    optimal_path_cost += 1.01*HoloObstacleHeuristic(next_node);
    next_node->SetHeuCost(optimal_path_cost);
}

double UnionPlanner::TrajCost(std::shared_ptr<Node3d> current_node,
                              std::shared_ptr<Node3d> next_node)
{
    // evaluate cost on the trajectory and add current cost
    double piecewise_cost = 0.0;
    // if (next_node->GetDirec()) {
    //     piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
    //                     step_size_ * traj_forward_penalty_;
    // } else {
    //     piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
    //                     step_size_ * traj_back_penalty_;
    // }
    // if (current_node->GetDirec() != next_node->GetDirec()) {
    //     piecewise_cost += traj_gear_switch_penalty_;
    // }
    piecewise_cost += traj_steer_penalty_ * std::fabs(next_node->GetSteer());
    piecewise_cost += traj_steer_change_penalty_ *
                        std::fabs(next_node->GetSteer() - current_node->GetSteer());
    piecewise_cost += traj_v_change_penalty_ * 
                        std::fabs(next_node->GetV() - current_node->GetV());
    return piecewise_cost;
}

double UnionPlanner::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node)
{
    double h;
    // h = std::fabs(next_node->GetX() - end_node_->GetX()) + std::fabs(next_node->GetY() - end_node_->GetY());
    h = std::sqrt(std::pow(next_node->GetX() - end_node_->GetX(), 2) + std::pow(next_node->GetY() - end_node_->GetY(), 2));
    return h;
}

bool UnionPlanner::GetResult(PlannerResult* result)
{
    std::shared_ptr<Node3d> current_node = final_node_;
    std::vector<double> result_x;
    std::vector<double> result_y;
    std::vector<double> result_phi;
    std::vector<double> result_v;
    std::vector<double> result_steer;
    while (current_node->GetPreNode() != nullptr) {
        // std::vector<double> x = current_node->GetXs();
        // std::vector<double> y = current_node->GetYs();
        // std::vector<double> phi = current_node->GetPhis();
        // if (x.empty() || y.empty() || phi.empty()) {
        // std::cout << "result size check failed" << std::endl;
        // return false;
        // }
        // if (x.size() != y.size() || x.size() != phi.size()) {
        // std::cout << "states sizes are not equal" << std::endl;
        // return false;
        // }
        // std::reverse(x.begin(), x.end());
        // std::reverse(y.begin(), y.end());
        // std::reverse(phi.begin(), phi.end());
        // x.pop_back();
        // y.pop_back();
        // phi.pop_back();
        // hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
        // hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
        // hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
        result_x.push_back(current_node->GetX());
        result_y.push_back(current_node->GetY());
        result_phi.push_back(current_node->GetPhi());
        result_v.push_back(current_node->GetV());
        result_steer.push_back(current_node->GetSteer());
        current_node = current_node->GetPreNode();
    }
    result_x.push_back(current_node->GetX());
    result_y.push_back(current_node->GetY());
    result_phi.push_back(current_node->GetPhi());
    result_v.push_back(current_node->GetV());
    result_steer.push_back(current_node->GetSteer());
    std::reverse(result_x.begin(), result_x.end());
    std::reverse(result_y.begin(), result_y.end());
    std::reverse(result_phi.begin(), result_phi.end());
    std::reverse(result_v.begin(), result_v.end());
    std::reverse(result_steer.begin(), result_steer.end());

    (*result).x = result_x;
    (*result).y = result_y;
    (*result).phi = result_phi;
    (*result).v = result_v;
    (*result).steer = result_steer;

    // if (!GetTemporalProfile(result)) {
    //     std::cout << "GetSpeedProfile from Hybrid Astar path fails" << std::endl;
    //     return false;
    // }

    if (result->x.size() != result->y.size() ||
        result->x.size() != result->v.size() ||
        result->x.size() != result->phi.size()) {
        std::cout << "state sizes not equal, "
                << "result->x.size(): " << result->x.size() << ", result->y.size()"
                << result->y.size() << ", result->phi.size()" << result->phi.size()
                << ", result->v.size()" << result->v.size() << std::endl;
        return false;
    }
    if (result->x.size() != result->steer.size()) {
        std::cout << "control sizes not equal or not right" << std::endl;
        std::cout << " acceleration size: " << result->v.size() << std::endl;
        std::cout << " steer size: " << result->steer.size() << std::endl;
        std::cout << " x size: " << result->x.size() << std::endl;
        return false;
    }
    return true;
}

// bool UnionPlanner::GetTemporalProfile(PlannerResult* result)
// {
//     //
// }

// bool UnionPlanner::GenerateSpeedAcceleration(PlannerResult* result)
// {
//     //
// }

// bool UnionPlanner::GenerateSCurveSpeedAcceleration(PlannerResult* result)
// {
//     //
// }

}//namespace optimizer