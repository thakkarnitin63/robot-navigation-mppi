#ifndef TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_GENERATOR_HPP_

#include <vector>
#include "RobotNavigation10x/types.hpp"

/**
 * Trajectory generation with trapezoidal velocity profiles.
 * Assigns timestamps to path points based on robot dynamics and velocity constraints.
 */
class TrajectoryGenerator
{
public:
    TrajectoryGenerator();

    /**
     * Generate a time-parameterized trajectory from a spatial path.
     * Uses trapezoidal velocity profile with acceleration and cruise phases.
     *
     * @param smooth_path Smoothed 2D path from PathSmoother
     * @param max_velocity Maximum cruise velocity in m/s
     * @param acceleration Acceleration and deceleration rate in m/s²
     * @return Trajectory with timestamps assigned to each point
     */
    std::vector<TrajectoryPoint> generate_trajectory(
        const std::vector<Point2D>& smooth_path, 
        double max_velocity,
        double acceleration);

private:
    // Calculate Euclidean distance between two points
    double dist(const Point2D& p1, const Point2D& p2);
};

#endif // TRAJECTORY_GENERATOR_HPP_