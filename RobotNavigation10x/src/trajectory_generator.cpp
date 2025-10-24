#include "RobotNavigation10x/trajectory_generator.hpp"
#include <cmath>
#include <stdexcept>

TrajectoryGenerator::TrajectoryGenerator() {}

// Calculate 2D Euclidean distance between points
double TrajectoryGenerator::dist(const Point2D& p1, const Point2D& p2)
{
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

// Generate time-parameterized trajectory with trapezoidal velocity profile
std::vector<TrajectoryPoint> TrajectoryGenerator::generate_trajectory(
    const std::vector<Point2D>& smooth_path, 
    double max_velocity,
    double acceleration)
{
    std::vector<TrajectoryPoint> trajectory;
    if (smooth_path.size() < 2) {
        return trajectory;
    }

    // Calculate total path length
    double total_distance = 0.0;
    for (size_t i = 0; i < smooth_path.size() - 1; ++i) {
        total_distance += this->dist(smooth_path[i], smooth_path[i+1]);
    }

    if (total_distance < 1e-6) {
        trajectory.push_back({smooth_path[0].x, smooth_path[0].y, 0.0});
        return trajectory;
    }

    // Calculate acceleration and deceleration distances
    // Using kinematic equation: d = v² / (2a)
    double dist_accel_phase = (max_velocity * max_velocity) / (2.0 * acceleration);
    double dist_decel_phase = dist_accel_phase;

    double time_accel_phase = 0.0, time_cruise_phase = 0.0, time_decel_phase = 0.0;
    double dist_cruise_phase = 0.0;
    double total_time = 0.0;

    // Determine velocity profile type
    if (dist_accel_phase + dist_decel_phase > total_distance)
    {
        // Triangular profile: path too short to reach max velocity
        dist_accel_phase = total_distance / 2.0;
        dist_decel_phase = total_distance / 2.0;
        dist_cruise_phase = 0.0;

        // Calculate peak velocity: v = sqrt(2ad)
        double peak_velocity = std::sqrt(2.0 * acceleration * dist_accel_phase);
        time_accel_phase = peak_velocity / acceleration;
        time_decel_phase = time_accel_phase;
        time_cruise_phase = 0.0;
    }
    else
    {
        // Trapezoidal profile: accelerate, cruise, decelerate
        dist_cruise_phase = total_distance - dist_accel_phase - dist_decel_phase;
        time_accel_phase = max_velocity / acceleration;
        time_decel_phase = time_accel_phase;
        time_cruise_phase = dist_cruise_phase / max_velocity;
    }

    total_time = time_accel_phase + time_cruise_phase + time_decel_phase;

    // Assign timestamps to each point based on distance traveled
    trajectory.push_back({smooth_path[0].x, smooth_path[0].y, 0.0});
    double distance_traveled = 0.0;

    for (size_t i = 0; i < smooth_path.size() - 1; ++i)
    {
        double segment_distance = this->dist(smooth_path[i], smooth_path[i+1]);
        distance_traveled += segment_distance;
        double current_time = 0.0;

        if (distance_traveled <= dist_accel_phase)
        {
            // Acceleration phase: t = sqrt(2d/a)
            current_time = std::sqrt(2.0 * distance_traveled / acceleration);
        }
        else if (distance_traveled <= dist_accel_phase + dist_cruise_phase)
        {
            // Cruise phase: constant velocity
            double dist_in_cruise = distance_traveled - dist_accel_phase;
            current_time = time_accel_phase + (dist_in_cruise / max_velocity);
        }
        else
        {
            // Deceleration phase: compute time backwards from end
            double dist_from_end = total_distance - distance_traveled;
            double time_from_end = std::sqrt(2.0 * dist_from_end / acceleration);
            current_time = total_time - time_from_end;
        }

        trajectory.push_back({smooth_path[i+1].x, smooth_path[i+1].y, current_time});
    }

    return trajectory;
}