/**
 * MPPI Trajectory Tracker Node
 * 
 * Implements Model Predictive Path Integral control with hybrid obstacle detection.
 * Combines static obstacle knowledge from the world file with real-time LiDAR sensing.
 */

#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <vector>
#include <memory>
#include <random>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "RobotNavigation10x/types.hpp"
#include "RobotNavigation10x/path_smoother.hpp"
#include "RobotNavigation10x/trajectory_generator.hpp"
#include "RobotNavigation10x/mppi_controller.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MPPITrackerNode : public rclcpp::Node
{
public:
    MPPITrackerNode() : Node("mppi_tracker"), is_initialized_(false)
    {
        // Declare and load ROS parameters
        this->declare_parameter<double>("max_linear_velocity", 0.22);
        this->declare_parameter<double>("max_acceleration", 0.5);
        this->declare_parameter<double>("spline_tension", 0.31);
        this->declare_parameter<int>("points_per_segment", 3);
        this->declare_parameter<int>("num_waypoints", 8);
        this->declare_parameter<double>("waypoint_offset", 2.0);
        this->declare_parameter<double>("controller_frequency", 20.0);
        
        this->declare_parameter<int>("mppi_num_samples", 1000);
        this->declare_parameter<int>("mppi_horizon_steps", 30);
        this->declare_parameter<double>("mppi_dt", 0.05);
        this->declare_parameter<double>("mppi_lambda", 1.0);
        this->declare_parameter<double>("mppi_weight_tracking", 10.0);
        this->declare_parameter<double>("mppi_weight_static_obstacle", 150.0);
        this->declare_parameter<double>("mppi_weight_lidar_obstacle", 100.0);
        this->declare_parameter<double>("mppi_weight_control", 0.1);
        this->declare_parameter<double>("mppi_weight_smoothness", 1.0);
        this->declare_parameter<double>("mppi_weight_goal", 50.0);

        double max_linear_velocity = this->get_parameter("max_linear_velocity").as_double();
        double max_acceleration = this->get_parameter("max_acceleration").as_double();
        double spline_tension = this->get_parameter("spline_tension").as_double();
        int points_per_segment = this->get_parameter("points_per_segment").as_int();
        double controller_frequency = this->get_parameter("controller_frequency").as_double();
        int num_waypoints = this->get_parameter("num_waypoints").as_int();
        double waypoint_offset = this->get_parameter("waypoint_offset").as_double();

        RCLCPP_INFO(this->get_logger(), "===========================================");
        RCLCPP_INFO(this->get_logger(), "  MPPI Trajectory Tracker");
        RCLCPP_INFO(this->get_logger(), "===========================================");

        // Generate waypoints for obstacle world
        double world_bound = 4.0;
        std::vector<Point2D> waypoints = generate_sensible_waypoints(
            num_waypoints, -world_bound, -world_bound, world_bound, world_bound, 
            waypoint_offset, world_bound);
        RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints", waypoints.size());

        // Apply path smoothing
        PathSmoother smoother;
        std::vector<Point2D> smooth_path = smoother.generate_smooth_path(
            waypoints, points_per_segment, spline_tension);
        RCLCPP_INFO(this->get_logger(), "Smoothed path: %zu points", smooth_path.size());

        // Generate time-stamped trajectory
        TrajectoryGenerator traj_gen;
        this->trajectory_ = traj_gen.generate_trajectory(
            smooth_path, max_linear_velocity, max_acceleration);
        RCLCPP_INFO(this->get_logger(), "Trajectory: %zu points", trajectory_.size());

        // Configure MPPI controller
        MPPIController::MPPIParams mppi_params;
        mppi_params.num_samples = this->get_parameter("mppi_num_samples").as_int();
        mppi_params.horizon_steps = this->get_parameter("mppi_horizon_steps").as_int();
        mppi_params.dt = this->get_parameter("mppi_dt").as_double();
        mppi_params.max_linear_vel = max_linear_velocity;
        mppi_params.max_angular_vel = 2.84;
        mppi_params.lambda = this->get_parameter("mppi_lambda").as_double();
        mppi_params.weight_tracking = this->get_parameter("mppi_weight_tracking").as_double();
        mppi_params.weight_static_obstacle = this->get_parameter("mppi_weight_static_obstacle").as_double();
        mppi_params.weight_lidar_obstacle = this->get_parameter("mppi_weight_lidar_obstacle").as_double();
        mppi_params.weight_control = this->get_parameter("mppi_weight_control").as_double();
        mppi_params.weight_smoothness = this->get_parameter("mppi_weight_smoothness").as_double();
        mppi_params.weight_goal = this->get_parameter("mppi_weight_goal").as_double();

        this->mppi_controller_ = std::make_shared<MPPIController>(mppi_params);
        
        // Load static obstacles from environment
        std::vector<StaticObstacle> obstacles = load_obstacles_from_world();
        mppi_controller_->set_static_obstacles(obstacles);
        
        RCLCPP_INFO(this->get_logger(), "MPPI ready: K=%d, T=%d, obstacles=%zu",
                    mppi_params.num_samples, mppi_params.horizon_steps, obstacles.size());

        // Setup ROS publishers and subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        predicted_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mppi/predicted_path", 10);
        obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mppi/obstacles", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MPPITrackerNode::odom_callback, this, _1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MPPITrackerNode::scan_callback, this, _1));

        auto controller_period = std::chrono::duration<double>(1.0 / controller_frequency);
        timer_ = this->create_wall_timer(
            controller_period, std::bind(&MPPITrackerNode::control_loop, this));

        obstacle_marker_timer_ = this->create_wall_timer(
            1s, std::bind(&MPPITrackerNode::publish_obstacle_markers, this));

        RCLCPP_INFO(this->get_logger(), "Waiting for odometry and laser scan...");
    }

private:
    // Load obstacle positions from nav_world_obst.world
    std::vector<StaticObstacle> load_obstacles_from_world()
    {
        std::vector<StaticObstacle> obstacles;
        
        // Box obstacle at (3.0, 2.0)
        StaticObstacle box1;
        box1.type = StaticObstacle::BOX;
        box1.x = 3.0;
        box1.y = 2.0;
        box1.size_x = 1.0;
        box1.size_y = 1.0;
        box1.rotation = 0.0;
        obstacles.push_back(box1);
        
        // Cylinder obstacle at (-1.0, 3.0)
        StaticObstacle cyl1;
        cyl1.type = StaticObstacle::CYLINDER;
        cyl1.x = -1.0;
        cyl1.y = 3.0;
        cyl1.radius = 0.5;
        cyl1.rotation = 0.0;
        obstacles.push_back(cyl1);
        
        // Box obstacle at (-3.0, -2.0)
        StaticObstacle box2;
        box2.type = StaticObstacle::BOX;
        box2.x = -3.0;
        box2.y = -2.0;
        box2.size_x = 1.0;
        box2.size_y = 1.0;
        box2.rotation = 0.0;
        obstacles.push_back(box2);
        
        // Cylinder obstacle at (1.0, -3.0)
        StaticObstacle cyl2;
        cyl2.type = StaticObstacle::CYLINDER;
        cyl2.x = 1.0;
        cyl2.y = -3.0;
        cyl2.radius = 0.5;
        cyl2.rotation = 0.0;
        obstacles.push_back(cyl2);
        
        // Small box obstacle at (-3.0, 1.0)
        StaticObstacle box3;
        box3.type = StaticObstacle::BOX;
        box3.x = -3.0;
        box3.y = 1.0;
        box3.size_x = 0.5;
        box3.size_y = 0.5;
        box3.rotation = 0.0;
        obstacles.push_back(box3);
        
        // Small cylinder obstacle at (3.0, -1.0)
        StaticObstacle cyl3;
        cyl3.type = StaticObstacle::CYLINDER;
        cyl3.x = 3.0;
        cyl3.y = -1.0;
        cyl3.radius = 0.2;
        cyl3.rotation = 0.0;
        obstacles.push_back(cyl3);
        
        // Large rotated box obstacle at center
        StaticObstacle box4;
        box4.type = StaticObstacle::BOX;
        box4.x = 0.0;
        box4.y = 0.0;
        box4.size_x = 1.5;
        box4.size_y = 1.5;
        box4.rotation = 0.785;  // 45 degrees
        obstacles.push_back(box4);
        
        return obstacles;
    }

    // Check if a point collides with any known obstacle
    bool is_point_in_obstacle(double x, double y, const std::vector<StaticObstacle>& obstacles)
    {
        const double safety_margin = 0.5;
        
        for (const auto& obs : obstacles) {
            if (obs.type == StaticObstacle::BOX) {
                // Transform to box local frame for accurate collision check
                double dx = x - obs.x;
                double dy = y - obs.y;
                
                double cos_theta = std::cos(-obs.rotation);
                double sin_theta = std::sin(-obs.rotation);
                
                double local_x = dx * cos_theta - dy * sin_theta;
                double local_y = dx * sin_theta + dy * cos_theta;
                
                double half_x = obs.size_x / 2.0 + safety_margin;
                double half_y = obs.size_y / 2.0 + safety_margin;
                
                if (std::abs(local_x) < half_x && std::abs(local_y) < half_y) {
                    return true;
                }
            } else {
                // Simple distance check for cylinders
                double dx = x - obs.x;
                double dy = y - obs.y;
                double dist = std::sqrt(dx*dx + dy*dy);
                
                if (dist < (obs.radius + safety_margin)) {
                    return true;
                }
            }
        }
        
        return false;
    }

    // Publish obstacle markers for RViz visualization
    void publish_obstacle_markers()
    {
        auto obstacles = mppi_controller_->get_static_obstacles();
        visualization_msgs::msg::MarkerArray marker_array;
        
        int id = 0;
        for (const auto& obs : obstacles) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->now();
            marker.ns = "static_obstacles";
            marker.id = id++;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.lifetime = rclcpp::Duration(0, 0);
            
            marker.pose.position.x = obs.x;
            marker.pose.position.y = obs.y;
            marker.pose.position.z = 0.5;
            
            if (obs.type == StaticObstacle::BOX) {
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.scale.x = obs.size_x;
                marker.scale.y = obs.size_y;
                marker.scale.z = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 0.7;
                
                marker.pose.orientation.w = std::cos(obs.rotation / 2.0);
                marker.pose.orientation.z = std::sin(obs.rotation / 2.0);
            } else {
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.scale.x = obs.radius * 2.0;
                marker.scale.y = obs.radius * 2.0;
                marker.scale.z = 2.0;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.color.a = 0.7;
            }
            
            marker_array.markers.push_back(marker);
        }
        
        obstacle_marker_pub_->publish(marker_array);
    }

    // Generate waypoints with obstacle avoidance
    std::vector<Point2D> generate_sensible_waypoints(
        int n, double start_x, double start_y, double end_x, double end_y,
        double max_offset, double boundary)
    {
        std::vector<Point2D> waypoints;
        random_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<double> dist(-max_offset, max_offset);

        // Load obstacles to check for collisions
        std::vector<StaticObstacle> obstacles = load_obstacles_from_world();

        // Validate and add start point
        if (is_point_in_obstacle(start_x, start_y, obstacles)) {
            RCLCPP_WARN(this->get_logger(), "Start position in obstacle, adjusting");
            start_x += 0.5;
            start_y += 0.5;
        }
        waypoints.push_back({start_x, start_y});

        double dx = (end_x - start_x) / (n - 1);
        double dy = (end_y - start_y) / (n - 1);

        // Generate intermediate waypoints avoiding obstacles
        for (int i = 1; i < n - 1; ++i) {
            double ideal_x = start_x + i * dx;
            double ideal_y = start_y + i * dy;
            
            double new_x, new_y;
            int attempts = 0;
            const int max_attempts = 50;
            
            // Try to find obstacle-free position
            do {
                double offset_x = dist(random_generator_);
                double offset_y = dist(random_generator_);
                new_x = std::clamp(ideal_x + offset_x, -boundary, boundary);
                new_y = std::clamp(ideal_y + offset_y, -boundary, boundary);
                attempts++;
                
                if (attempts >= max_attempts) {
                    RCLCPP_WARN(this->get_logger(), 
                        "Waypoint %d: using ideal position after %d attempts", i, max_attempts);
                    new_x = ideal_x;
                    new_y = ideal_y;
                    break;
                }
            } while (is_point_in_obstacle(new_x, new_y, obstacles));
            
            waypoints.push_back({new_x, new_y});
        }

        // Validate and add goal point
        if (is_point_in_obstacle(end_x, end_y, obstacles)) {
            RCLCPP_WARN(this->get_logger(), "Goal position in obstacle, adjusting");
            end_x -= 0.5;
            end_y -= 0.5;
        }
        waypoints.push_back({end_x, end_y});
        
        return waypoints;
    }

    // Store current robot pose from odometry
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->current_pose_ = msg->pose.pose;
        this->current_velocity_ = msg->twist.twist;
        this->is_initialized_ = true;
    }

    // Store latest laser scan for obstacle detection
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        this->latest_scan_ = msg;
    }

    // Main control loop executed at controller frequency
    void control_loop()
    {
        if (!is_initialized_ || trajectory_.empty()) return;

        // Check if goal reached
        if (mppi_controller_->is_goal_reached(this->current_pose_, this->trajectory_))
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
            timer_->cancel();
            return;
        }

        // Compute optimal control using MPPI
        auto start_time = std::chrono::high_resolution_clock::now();
        
        geometry_msgs::msg::Twist cmd_vel = mppi_controller_->compute_control(
            this->current_pose_,
            this->current_velocity_,
            this->trajectory_,
            this->latest_scan_);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count();

        // Log computation time periodically
        static int counter = 0;
        if (++counter % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "MPPI computation: %ld ms", duration);
        }

        cmd_vel_pub_->publish(cmd_vel);
        publish_predicted_trajectory();
    }

    // Visualize MPPI predicted trajectory
    void publish_predicted_trajectory()
    {
        auto predicted_traj = mppi_controller_->get_predicted_trajectory();
        visualization_msgs::msg::MarkerArray marker_array;
        
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "odom";
        path_marker.header.stamp = this->now();
        path_marker.ns = "mppi_predicted_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.05;
        path_marker.color.r = 0.0;
        path_marker.color.g = 1.0;
        path_marker.color.b = 0.0;
        path_marker.color.a = 0.8;
        
        for (const auto& state : predicted_traj) {
            geometry_msgs::msg::Point p;
            p.x = state.x;
            p.y = state.y;
            p.z = 0.0;
            path_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(path_marker);
        predicted_path_pub_->publish(marker_array);
    }

    // Member variables
    std::vector<TrajectoryPoint> trajectory_;
    std::shared_ptr<MPPIController> mppi_controller_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr predicted_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr obstacle_marker_timer_;
    
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    bool is_initialized_;
    std::default_random_engine random_generator_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPPITrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}