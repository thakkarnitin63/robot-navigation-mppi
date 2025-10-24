#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <vector>
#include "RobotNavigation10x/types.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Twist;

/**
 * Pure Pursuit trajectory tracking controller.
 * Implements the classic geometric path following algorithm for differential drive robots.
 */
class TrajectoryTrackerController
{
public:
    /**
     * Initialize the Pure Pursuit controller.
     * 
     * @param lookahead_distance Distance ahead of robot to target (meters)
     * @param max_linear_velocity Maximum forward velocity (m/s)
     */
    TrajectoryTrackerController(double lookahead_distance, double max_linear_velocity);

    /**
     * Compute control commands to follow the trajectory.
     * Uses Pure Pursuit geometry to calculate required velocities.
     * 
     * @param current_pose Robot's current position and orientation
     * @param trajectory Time-stamped trajectory to follow
     * @return Velocity commands (linear and angular)
     */
    Twist calculate_control_command(
        const Pose& current_pose,
        const std::vector<TrajectoryPoint>& trajectory);

    /**
     * Check if robot has reached the trajectory endpoint.
     * 
     * @param current_pose Robot's current position
     * @param trajectory Trajectory being followed
     * @return True if within goal tolerance
     */
    bool is_goal_reached(
        const Pose& current_pose,
        const std::vector<TrajectoryPoint>& trajectory);

private:
    // Find the trajectory point closest to the robot
    size_t find_closest_point(
        const Pose& current_pose,
        const std::vector<TrajectoryPoint>& trajectory);

    // Find the lookahead point based on lookahead distance
    size_t find_goal_point(
        size_t closest_point_idx,
        const Pose& current_pose,
        const std::vector<TrajectoryPoint>& trajectory);

    // Extract yaw angle from quaternion orientation
    double get_yaw_from_quaternion(const Quaternion& q);

    // Calculate 2D Euclidean distance
    double dist(double x1, double y1, double x2, double y2);

    // Controller tuning parameters
    double lookahead_distance_;
    double max_linear_velocity_;
};

#endif // CONTROLLER_HPP_