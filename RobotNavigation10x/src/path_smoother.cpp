#include "RobotNavigation10x/path_smoother.hpp"
#include <cmath>

PathSmoother::PathSmoother() {}

// Generate smooth path through waypoints using Catmull-Rom splines
std::vector<Point2D> PathSmoother::generate_smooth_path(
    const std::vector<Point2D>& waypoints, 
    int points_per_segment, 
    double tension)
{
    std::vector<Point2D> smoothed_path;
    
    // Interpolate between each consecutive waypoint pair
    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        // Select four control points for Catmull-Rom spline
        Point2D p0 = (i == 0) ? waypoints[i] : waypoints[i - 1];
        Point2D p1 = waypoints[i];
        Point2D p2 = waypoints[i + 1];
        Point2D p3 = (i + 2 == waypoints.size()) ? waypoints[i + 1] : waypoints[i + 2];

        // Generate interpolated points along spline segment
        for (int j = 0; j < points_per_segment; ++j)
        {
            double t = (double)j / (double)points_per_segment;
            smoothed_path.push_back(this->catmull_rom_spline(p0, p1, p2, p3, t, tension));
        }
    }
    
    // Add final waypoint
    smoothed_path.push_back(waypoints.back());
    return smoothed_path;
}

// Catmull-Rom spline interpolation with configurable tension
Point2D PathSmoother::catmull_rom_spline(
    Point2D p0, Point2D p1, Point2D p2, Point2D p3, 
    double t, 
    double tension)
{
    Point2D result;
    double t2 = t * t;
    double t3 = t * t * t;

    // Compute tangent vectors based on neighboring points
    // Tension controls influence of outer control points
    double m1_x = tension * (p2.x - p0.x);
    double m1_y = tension * (p2.y - p0.y);
    double m2_x = tension * (p3.x - p1.x);
    double m2_y = tension * (p3.y - p1.y);

    // Hermite basis functions for cubic interpolation
    double h00 =  2*t3 - 3*t2 + 1;
    double h10 = -2*t3 + 3*t2;
    double h01 =    t3 - 2*t2 + t;
    double h11 =    t3 -   t2;

    // Combine basis functions with control points and tangents
    result.x = h00*p1.x + h10*p2.x + h01*m1_x + h11*m2_x;
    result.y = h00*p1.y + h10*p2.y + h01*m1_y + h11*m2_y;

    return result;
}