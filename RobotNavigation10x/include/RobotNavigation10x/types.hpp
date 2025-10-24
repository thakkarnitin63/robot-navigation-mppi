#ifndef ROBOT_NAV_TYPES_HPP_
#define ROBOT_NAV_TYPES_HPP_

/**
 * Basic 2D point structure for representing positions in space.
 * Used throughout the navigation system for waypoints and paths.
 */
struct Point2D {
    double x;  // X coordinate in meters
    double y;  // Y coordinate in meters
};

/**
 * Time-stamped trajectory point structure.
 * Extends Point2D with temporal information for trajectory generation.
 */
struct TrajectoryPoint {
    double x;           // X coordinate in meters
    double y;           // Y coordinate in meters
    double time_stamp;  // Time from trajectory start in seconds
};

#endif // ROBOT_NAV_TYPES_HPP_