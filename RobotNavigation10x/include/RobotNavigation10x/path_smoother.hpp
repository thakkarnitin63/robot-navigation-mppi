#ifndef PATH_SMOOTHER_HPP_
#define PATH_SMOOTHER_HPP_

#include <vector>
#include <string>
#include "RobotNavigation10x/types.hpp"

/**
 * Path smoothing using Catmull-Rom splines.
 * Converts discrete waypoints into a smooth, continuous path suitable for robot navigation.
 */
class PathSmoother
{
public:
    PathSmoother();
    
    /**
     * Generate a smooth path through the given waypoints using Catmull-Rom interpolation.
     * 
     * @param waypoints Input waypoints to connect
     * @param points_per_segment Number of interpolated points between each waypoint pair
     * @param tension Spline tension parameter (0.0 = tight curves, 0.5 = loose curves)
     * @return Smoothed path as a dense sequence of points
     */
    std::vector<Point2D> generate_smooth_path(
        const std::vector<Point2D>& waypoints, 
        int points_per_segment, 
        double tension = 0.5); 

private:
    /**
     * Catmull-Rom spline interpolation between four control points.
     * Uses Hermite basis functions with configurable tension.
     */
    Point2D catmull_rom_spline(
        Point2D p0, Point2D p1, Point2D p2, Point2D p3, 
        double t, 
        double tension);
};

#endif // PATH_SMOOTHER_HPP_