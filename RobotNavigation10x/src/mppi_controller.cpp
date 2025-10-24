#include "RobotNavigation10x/mppi_controller.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include "rclcpp/rclcpp.hpp"

MPPIController::MPPIController(const MPPIParams& params)
    : params_(params),
      noise_distribution_(0.0, 1.0)
{
    random_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
    
    // Initialize control sequence with zeros for warm starting
    prev_control_sequence_.resize(params_.horizon_steps);
    for (auto& ctrl : prev_control_sequence_) {
        ctrl.v = 0.0;
        ctrl.omega = 0.0;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("mppi"), 
        "MPPI initialized: samples=%d, horizon=%d, dt=%.3fs",
        params_.num_samples, params_.horizon_steps, params_.dt);
}

void MPPIController::set_static_obstacles(const std::vector<StaticObstacle>& obstacles)
{
    static_obstacles_ = obstacles;
    RCLCPP_INFO(rclcpp::get_logger("mppi"), 
        "Loaded %zu static obstacles", obstacles.size());
}

geometry_msgs::msg::Twist MPPIController::compute_control(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::Twist& current_velocity,
    const std::vector<TrajectoryPoint>& trajectory,
    const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan)
{

    if (trajectory.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("mppi"), "No trajectory to follow!");
        geometry_msgs::msg::Twist cmd_vel;
        return cmd_vel;
    }
    
    RobotState current_state = pose_to_state(current_pose, current_velocity);
    
    // Update dynamic obstacles from LiDAR
    if (laser_scan) {
        process_laser_scan(laser_scan, current_state);
    }
    
    // Prepare storage for sampling
    std::vector<std::vector<ControlInput>> sampled_controls(params_.num_samples);
    std::vector<std::vector<RobotState>> sampled_trajectories(params_.num_samples);
    std::vector<double> costs(params_.num_samples);
    
    // Generate sampled control sequences with exploration noise
    for (int k = 0; k < params_.num_samples; ++k) {
        sampled_controls[k] = generate_control_sequence(prev_control_sequence_);
    }
    
    // Simulate and evaluate each sampled trajectory in parallel
    #pragma omp parallel for
    for (int k = 0; k < params_.num_samples; ++k) {
        sampled_trajectories[k] = rollout_trajectory(current_state, sampled_controls[k]);
        costs[k] = compute_trajectory_cost(
            sampled_trajectories[k], 
            sampled_controls[k], 
            trajectory,
            current_state);
    }
    
    // Find minimum cost for numerical stability in exponential weighting
    double min_cost = *std::min_element(costs.begin(), costs.end());
    
    // Compute importance weights using exponential transformation
    std::vector<double> weights(params_.num_samples);
    double weight_sum = 0.0;
    
    for (int k = 0; k < params_.num_samples; ++k) {
        weights[k] = std::exp(-params_.lambda * (costs[k] - min_cost));
        weight_sum += weights[k];
    }
    
    // Normalize weights
    for (int k = 0; k < params_.num_samples; ++k) {
        weights[k] /= weight_sum;
    }
    
    // Compute weighted average of all sampled control sequences
    std::vector<ControlInput> optimal_sequence(params_.horizon_steps);
    
    for (int t = 0; t < params_.horizon_steps; ++t) {
        optimal_sequence[t].v = 0.0;
        optimal_sequence[t].omega = 0.0;
        
        for (int k = 0; k < params_.num_samples; ++k) {
            optimal_sequence[t].v += weights[k] * sampled_controls[k][t].v;
            optimal_sequence[t].omega += weights[k] * sampled_controls[k][t].omega;
        }
    }
    
    // Save optimal sequence for warm starting next iteration
    prev_control_sequence_ = optimal_sequence;
    best_trajectory_ = rollout_trajectory(current_state, optimal_sequence);
    
    // Return first control in optimal sequence
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = clamp(optimal_sequence[0].v, 0.0, params_.max_linear_vel);
    cmd_vel.angular.z = clamp(optimal_sequence[0].omega, 
                               -params_.max_angular_vel, 
                               params_.max_angular_vel);
    
    return cmd_vel;
}

std::vector<MPPIController::ControlInput> MPPIController::generate_control_sequence(
    const std::vector<ControlInput>& base_sequence)
{
    std::vector<ControlInput> noisy_sequence(params_.horizon_steps);
    
    for (int t = 0; t < params_.horizon_steps; ++t) {
        // Sample Gaussian noise for exploration
        double v_noise = noise_distribution_(random_generator_) * params_.linear_vel_noise_std;
        double omega_noise = noise_distribution_(random_generator_) * params_.angular_vel_noise_std;
        
        noisy_sequence[t].v = base_sequence[t].v + v_noise;
        noisy_sequence[t].omega = base_sequence[t].omega + omega_noise;
        
        // Apply acceleration constraints
        if (t > 0) {
            double dv = noisy_sequence[t].v - noisy_sequence[t-1].v;
            double max_dv = params_.max_linear_accel * params_.dt;
            noisy_sequence[t].v = noisy_sequence[t-1].v + clamp(dv, -max_dv, max_dv);
            
            double domega = noisy_sequence[t].omega - noisy_sequence[t-1].omega;
            double max_domega = params_.max_angular_accel * params_.dt;
            noisy_sequence[t].omega = noisy_sequence[t-1].omega + clamp(domega, -max_domega, max_domega);
        }
        
        // Apply velocity limits
        noisy_sequence[t].v = clamp(noisy_sequence[t].v, 0.0, params_.max_linear_vel);
        noisy_sequence[t].omega = clamp(noisy_sequence[t].omega, 
                                        -params_.max_angular_vel, 
                                        params_.max_angular_vel);
    }
    
    return noisy_sequence;
}

std::vector<MPPIController::RobotState> MPPIController::rollout_trajectory(
    const RobotState& initial_state,
    const std::vector<ControlInput>& controls)
{
    std::vector<RobotState> trajectory;
    trajectory.reserve(params_.horizon_steps + 1);
    trajectory.push_back(initial_state);
    
    RobotState state = initial_state;
    for (int t = 0; t < params_.horizon_steps; ++t) {
        state = simulate_step(state, controls[t], params_.dt);
        trajectory.push_back(state);
    }
    
    return trajectory;
}

MPPIController::RobotState MPPIController::simulate_step(
    const RobotState& state,
    const ControlInput& control,
    double dt)
{
    RobotState next_state;
    
    // Differential drive kinematics
    next_state.theta = normalize_angle(state.theta + control.omega * dt);
    next_state.x = state.x + control.v * std::cos(state.theta) * dt;
    next_state.y = state.y + control.v * std::sin(state.theta) * dt;
    next_state.v = control.v;
    next_state.omega = control.omega;
    
    return next_state;
}

double MPPIController::compute_trajectory_cost(
    const std::vector<RobotState>& trajectory,
    const std::vector<ControlInput>& controls,
    const std::vector<TrajectoryPoint>& reference,
    const RobotState& current_state)
{
    double total_cost = 0.0;
    
    total_cost += params_.weight_tracking * compute_tracking_cost(trajectory, reference);
    total_cost += params_.weight_static_obstacle * compute_static_obstacle_cost(trajectory);
    total_cost += params_.weight_lidar_obstacle * compute_lidar_obstacle_cost(trajectory);
    total_cost += params_.weight_control * compute_control_cost(controls);
    total_cost += params_.weight_smoothness * compute_smoothness_cost(controls);
    total_cost += params_.weight_goal * compute_goal_cost(trajectory.back(), reference);
    
    return total_cost;
}

double MPPIController::compute_tracking_cost(
    const std::vector<RobotState>& trajectory,
    const std::vector<TrajectoryPoint>& reference)
{
    if (reference.empty()) return 0.0;
    
    double cost = 0.0;
    for (const auto& state : trajectory) {
        size_t closest_idx = find_closest_trajectory_point(state, reference);
        double dx = state.x - reference[closest_idx].x;
        double dy = state.y - reference[closest_idx].y;
        cost += dx*dx + dy*dy;
    }
    
    return cost / trajectory.size();
}

double MPPIController::compute_static_obstacle_cost(
    const std::vector<RobotState>& trajectory)
{
    if (static_obstacles_.empty()) return 0.0;
    
    double cost = 0.0;
    
    for (const auto& state : trajectory) {
        for (const auto& obstacle : static_obstacles_) {
            bool collision = (obstacle.type == StaticObstacle::BOX) ?
                is_colliding_with_box(state.x, state.y, obstacle) :
                is_colliding_with_cylinder(state.x, state.y, obstacle);
            
            if (collision) {
                cost += 10000.0;
            } else {
                double dist = distance_to_static_obstacle(state.x, state.y);
                if (dist < params_.danger_distance) {
                    double clearance = std::max(dist - params_.robot_radius, 0.01);
                    double normalized = clearance / (params_.danger_distance - params_.robot_radius);
                    cost += std::exp(-5.0 * normalized);
                }
            }
        }
    }
    
    return cost;
}

double MPPIController::compute_lidar_obstacle_cost(
    const std::vector<RobotState>& trajectory)
{
    if (lidar_obstacles_.empty()) return 0.0;
    
    double cost = 0.0;
    
    for (const auto& state : trajectory) {
        double min_dist = distance_to_lidar_obstacle(state.x, state.y);
        
        if (min_dist < params_.robot_radius) {
            cost += 5000.0;
        } else if (min_dist < params_.danger_distance) {
            double clearance = min_dist - params_.robot_radius;
            double normalized = clearance / (params_.danger_distance - params_.robot_radius);
            cost += std::exp(-4.0 * normalized);
        }
    }
    
    return cost;
}

bool MPPIController::is_colliding_with_box(
    double robot_x, double robot_y,
    const StaticObstacle& box)
{
    // Transform to box local coordinates
    double dx = robot_x - box.x;
    double dy = robot_y - box.y;
    
    double cos_theta = std::cos(-box.rotation);
    double sin_theta = std::sin(-box.rotation);
    
    double local_x = dx * cos_theta - dy * sin_theta;
    double local_y = dx * sin_theta + dy * cos_theta;
    
    // Check if inside expanded box
    double half_x = box.size_x / 2.0 + params_.robot_radius;
    double half_y = box.size_y / 2.0 + params_.robot_radius;
    
    return (std::abs(local_x) < half_x) && (std::abs(local_y) < half_y);
}

bool MPPIController::is_colliding_with_cylinder(
    double robot_x, double robot_y,
    const StaticObstacle& cylinder)
{
    double dx = robot_x - cylinder.x;
    double dy = robot_y - cylinder.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    
    return dist < (cylinder.radius + params_.robot_radius);
}

double MPPIController::distance_to_static_obstacle(double x, double y)
{
    if (static_obstacles_.empty()) return 1000.0;
    
    double min_dist = 1000.0;
    
    for (const auto& obstacle : static_obstacles_) {
        double dist = 0.0;
        
        if (obstacle.type == StaticObstacle::CYLINDER) {
            double dx = x - obstacle.x;
            double dy = y - obstacle.y;
            dist = std::sqrt(dx*dx + dy*dy) - obstacle.radius;
        } else {
            double dx = x - obstacle.x;
            double dy = y - obstacle.y;
            dist = std::sqrt(dx*dx + dy*dy) - std::max(obstacle.size_x, obstacle.size_y) / 2.0;
        }
        
        min_dist = std::min(min_dist, dist);
    }
    
    return min_dist;
}

double MPPIController::distance_to_lidar_obstacle(double x, double y)
{
    if (lidar_obstacles_.empty()) return 1000.0;
    
    double min_dist = 1000.0;
    
    for (const auto& obstacle : lidar_obstacles_) {
        double dx = x - obstacle.x;
        double dy = y - obstacle.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        min_dist = std::min(min_dist, dist);
    }
    
    return min_dist;
}

void MPPIController::process_laser_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan,
    const RobotState& current_state)
{
    lidar_obstacles_.clear();
    
    if (!scan || scan->ranges.empty()) return;
    
    // Convert laser readings to world frame obstacle points
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double range = scan->ranges[i];
        
        if (range < scan->range_min || range > scan->range_max || std::isnan(range)) {
            continue;
        }
        
        double angle = scan->angle_min + i * scan->angle_increment;
        double world_angle = current_state.theta + angle;
        
        Point2D obstacle;
        obstacle.x = current_state.x + range * std::cos(world_angle);
        obstacle.y = current_state.y + range * std::sin(world_angle);
        
        lidar_obstacles_.push_back(obstacle);
    }
}

double MPPIController::compute_control_cost(const std::vector<ControlInput>& controls)
{
    double cost = 0.0;
    for (const auto& ctrl : controls) {
        cost += ctrl.v * ctrl.v + ctrl.omega * ctrl.omega;
    }
    return cost / controls.size();
}

double MPPIController::compute_smoothness_cost(const std::vector<ControlInput>& controls)
{
    if (controls.size() < 2) return 0.0;
    
    double cost = 0.0;
    for (size_t t = 1; t < controls.size(); ++t) {
        double dv = controls[t].v - controls[t-1].v;
        double domega = controls[t].omega - controls[t-1].omega;
        cost += dv*dv + domega*domega;
    }
    return cost / (controls.size() - 1);
}

double MPPIController::compute_goal_cost(
    const RobotState& final_state,
    const std::vector<TrajectoryPoint>& reference)
{
    if (reference.empty()) return 0.0;
    
    const auto& goal = reference.back();
    double dx = final_state.x - goal.x;
    double dy = final_state.y - goal.y;
    return dx*dx + dy*dy;
}

size_t MPPIController::find_closest_trajectory_point(
    const RobotState& state,
    const std::vector<TrajectoryPoint>& trajectory)
{
    double min_dist = 1e9;
    size_t closest_idx = 0;
    
    for (size_t i = 0; i < trajectory.size(); ++i) {
        double dx = state.x - trajectory[i].x;
        double dy = state.y - trajectory[i].y;
        double dist = dx*dx + dy*dy;
        
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    
    return closest_idx;
}

bool MPPIController::is_goal_reached(
    const geometry_msgs::msg::Pose& current_pose,
    const std::vector<TrajectoryPoint>& trajectory)
{
    if (trajectory.empty()) return true;
    
    const auto& goal = trajectory.back();
    double dx = current_pose.position.x - goal.x;
    double dy = current_pose.position.y - goal.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    
    return dist < 0.15;
}

MPPIController::RobotState MPPIController::pose_to_state(
    const geometry_msgs::msg::Pose& pose,
    const geometry_msgs::msg::Twist& velocity)
{
    RobotState state;
    state.x = pose.position.x;
    state.y = pose.position.y;
    state.theta = get_yaw_from_quaternion(pose.orientation);
    state.v = velocity.linear.x;
    state.omega = velocity.angular.z;
    return state;
}

double MPPIController::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
    double t3 = 2.0 * (q.w * q.z + q.x * q.y);
    double t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(t3, t4);
}

double MPPIController::normalize_angle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}