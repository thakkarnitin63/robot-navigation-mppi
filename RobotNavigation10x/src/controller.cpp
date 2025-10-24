#include "RobotNavigation10x/controller.hpp"
#include <cmath>
#include "rclcpp/rclcpp.hpp"

// Calculate 2D Euclidean distance between two points
double TrajectoryTrackerController::dist(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

// Extract yaw angle from quaternion orientation
double TrajectoryTrackerController::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
    double t3 = 2.0 * (q.w * q.z + q.x * q.y);
    double t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(t3, t4);
}

// Initialize controller with tuning parameters
TrajectoryTrackerController::TrajectoryTrackerController(double lookahead_distance, double max_linear_velocity)
    : lookahead_distance_(lookahead_distance),
      max_linear_velocity_(max_linear_velocity)
{
    RCLCPP_INFO(rclcpp::get_logger("controller_lib"), 
        "Pure Pursuit initialized: lookahead=%.2fm, max_vel=%.2fm/s",
        lookahead_distance_, max_linear_velocity_);
}

// Find the trajectory point closest to the robot
size_t TrajectoryTrackerController::find_closest_point(
    const geometry_msgs::msg::Pose& current_pose,
    const std::vector<TrajectoryPoint>& trajectory)
{
    double min_dist = 1e9;
    size_t closest_idx = 0;

    for (size_t i = 0; i < trajectory.size(); ++i) {
        double d = this->dist(
            current_pose.position.x, current_pose.position.y,
            trajectory[i].x, trajectory[i].y);
        
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }
    return closest_idx;
}

// Find the lookahead goal point along the trajectory
size_t TrajectoryTrackerController::find_goal_point(
    size_t closest_point_idx,
    const geometry_msgs::msg::Pose& current_pose,
    const std::vector<TrajectoryPoint>& trajectory)
{
    // Search forward from the closest point
    for (size_t i = closest_point_idx; i < trajectory.size(); ++i)
    {
        double d = this->dist(
            current_pose.position.x, current_pose.position.y,
            trajectory[i].x, trajectory[i].y);
        
        // Use the first point beyond lookahead distance
        if (d > lookahead_distance_) {
            return i;
        }
    }
    
    // Near path end: target the final point
    return trajectory.size() - 1;
}

// Check if robot has reached the trajectory endpoint
bool TrajectoryTrackerController::is_goal_reached(
    const geometry_msgs::msg::Pose& current_pose,
    const std::vector<TrajectoryPoint>& trajectory)
{
    if (trajectory.empty()) return true;

    double goal_x = trajectory.back().x;
    double goal_y = trajectory.back().y;
    double d = this->dist(current_pose.position.x, current_pose.position.y, goal_x, goal_y);

    return d < 0.1;  // 10cm tolerance
}

// Compute velocity commands using Pure Pursuit algorithm
geometry_msgs::msg::Twist TrajectoryTrackerController::calculate_control_command(
    const geometry_msgs::msg::Pose& current_pose,
    const std::vector<TrajectoryPoint>& trajectory)
{
    geometry_msgs::msg::Twist cmd_vel;

    if (trajectory.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("controller"), "Empty trajectory!");
        return cmd_vel;  // Return zero velocity
    }

    // Identify target point on trajectory
    size_t closest_idx = this->find_closest_point(current_pose, trajectory);
    size_t goal_idx = this->find_goal_point(closest_idx, current_pose, trajectory);
    
    double goal_x = trajectory[goal_idx].x;
    double goal_y = trajectory[goal_idx].y;

    // Get robot state
    double robot_x = current_pose.position.x;
    double robot_y = current_pose.position.y;
    double robot_yaw = this->get_yaw_from_quaternion(current_pose.orientation);

    // Pure Pursuit geometry
    // Angle from robot heading to goal point
    double alpha = std::atan2(goal_y - robot_y, goal_x - robot_x) - robot_yaw;
    
    // Distance to goal point (chord length)
    double Ld = this->dist(robot_x, robot_y, goal_x, goal_y);

    if (Ld < 0.01) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
    }

    // Curvature of circular arc to intercept goal
    // Pure Pursuit formula: k = 2*sin(alpha) / Ld
    double k = (2.0 * std::sin(alpha)) / Ld;

    // Set velocities
    cmd_vel.linear.x = max_linear_velocity_;
    cmd_vel.angular.z = max_linear_velocity_ * k;  // ω = v * k

    return cmd_vel;
}