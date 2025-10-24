/**
 * Pure Pursuit Trajectory Tracker Node
 * 
 * Implements the complete navigation pipeline:
 * 1. Generates waypoints
 * 2. Applies path smoothing (Catmull-Rom splines)
 * 3. Creates time-parameterized trajectory
 * 4. Executes Pure Pursuit control
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

#include "RobotNavigation10x/types.hpp"
#include "RobotNavigation10x/path_smoother.hpp"
#include "RobotNavigation10x/trajectory_generator.hpp"
#include "RobotNavigation10x/controller.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class TrajectoryTracker : public rclcpp::Node
{
public:
    TrajectoryTracker() : Node("trajectory_tracker"), is_initialized_(false)
    {
        // Declare ROS parameters
        this->declare_parameter<double>("max_linear_velocity");
        this->declare_parameter<double>("max_acceleration");
        this->declare_parameter<int>("num_waypoints");
        this->declare_parameter<double>("waypoint_offset");
        this->declare_parameter<double>("spline_tension");
        this->declare_parameter<int>("points_per_segment");
        this->declare_parameter<double>("lookahead_distance");
        this->declare_parameter<double>("controller_frequency");

        // Load parameters from YAML file
        double max_linear_velocity = this->get_parameter("max_linear_velocity").as_double();
        double max_acceleration = this->get_parameter("max_acceleration").as_double();
        double spline_tension = this->get_parameter("spline_tension").as_double();
        int points_per_segment = this->get_parameter("points_per_segment").as_int();
        double lookahead_distance = this->get_parameter("lookahead_distance").as_double();
        double controller_frequency = this->get_parameter("controller_frequency").as_double();
        int num_waypoints = this->get_parameter("num_waypoints").as_int();
        double waypoint_offset = this->get_parameter("waypoint_offset").as_double();

        RCLCPP_INFO(this->get_logger(), "===========================================");
        RCLCPP_INFO(this->get_logger(), "  Pure Pursuit Trajectory Tracker");
        RCLCPP_INFO(this->get_logger(), "===========================================");
        RCLCPP_INFO(this->get_logger(), "Max velocity: %.2f m/s", max_linear_velocity);
        RCLCPP_INFO(this->get_logger(), "Lookahead: %.2f m", lookahead_distance);
        RCLCPP_INFO(this->get_logger(), "Generating %d waypoints...", num_waypoints);

        // Generate waypoints
        double world_bound = 4.0;
        std::vector<Point2D> waypoints = this->generate_sensible_waypoints(
            num_waypoints,
            -world_bound, -world_bound,
            world_bound, world_bound,
            waypoint_offset,
            world_bound
        );
        
        if (waypoints.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Insufficient waypoints generated!");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints", waypoints.size());
        save_path_to_csv(waypoints, "original_waypoints.csv");

        // Task 1: Path Smoothing
        PathSmoother smoother;
        std::vector<Point2D> smooth_path = smoother.generate_smooth_path(
            waypoints, points_per_segment, spline_tension);
        
        if (smooth_path.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Path smoothing failed!");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Smoothed path: %zu points", smooth_path.size());
        save_path_to_csv(smooth_path, "smooth_path.csv");

        // Task 2: Trajectory Generation
        TrajectoryGenerator traj_gen;
        this->trajectory_ = traj_gen.generate_trajectory(
            smooth_path, max_linear_velocity, max_acceleration);
        
        if (trajectory_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory generation failed!");
            rclcpp::shutdown();
            return;
        }
        
        if (trajectory_.size() < 3) {
            RCLCPP_WARN(this->get_logger(), 
                "Trajectory very short (%zu points). May not be smooth.", trajectory_.size());
        }

        if (trajectory_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory generation failed!");
            rclcpp::shutdown();
            return;
        }  
        
        RCLCPP_INFO(this->get_logger(), "Trajectory: %zu points", trajectory_.size());
        save_trajectory_to_csv(this->trajectory_, "trajectory.csv");

        if (waypoints.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Need at least 2 waypoints!");
            rclcpp::shutdown();
            return;
        }
        
        if (!trajectory_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Total time: %.2f seconds", 
                        trajectory_.back().time_stamp);
        }

        // Task 3: Initialize Controller
        this->controller_ = std::make_shared<TrajectoryTrackerController>(
            lookahead_distance, max_linear_velocity);

        // Setup ROS publishers and subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrajectoryTracker::odom_callback, this, _1));

        auto controller_period = std::chrono::duration<double>(1.0 / controller_frequency);
        timer_ = this->create_wall_timer(
            controller_period, std::bind(&TrajectoryTracker::controller_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Ready! Waiting for odometry...");
    }

private:
    // Generate waypoints progressing from start to end with random variation
    std::vector<Point2D> generate_sensible_waypoints(
        int n, double start_x, double start_y, double end_x, double end_y, 
        double max_offset, double boundary)
    {
        std::vector<Point2D> waypoints;
        random_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<double> dist(-max_offset, max_offset);

        waypoints.push_back({start_x, start_y});

        double dx = (end_x - start_x) / (n - 1);
        double dy = (end_y - start_y) / (n - 1);

        for (int i = 1; i < n - 1; ++i)
        {
            double ideal_x = start_x + i * dx;
            double ideal_y = start_y + i * dy;
            double offset_x = dist(random_generator_);
            double offset_y = dist(random_generator_);
            double new_x = std::clamp(ideal_x + offset_x, -boundary, boundary);
            double new_y = std::clamp(ideal_y + offset_y, -boundary, boundary);
            waypoints.push_back({new_x, new_y});
        }

        waypoints.push_back({end_x, end_y});
        return waypoints;
    }

    // Store current robot pose from odometry
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->current_pose_ = msg->pose.pose;
        this->is_initialized_ = true;
    }

    // Main control loop
    void controller_loop()
    {
        if (!is_initialized_ || trajectory_.empty()) {
            return;
        }

        if (controller_->is_goal_reached(this->current_pose_, this->trajectory_))
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
            timer_->cancel();
            return;
        }

        geometry_msgs::msg::Twist cmd_vel = controller_->calculate_control_command(
            this->current_pose_, 
            this->trajectory_);
        
        cmd_vel_pub_->publish(cmd_vel);
    }

    // Save 2D path to CSV file in current directory
    void save_path_to_csv(const std::vector<Point2D>& path, const std::string& filename)
    {
        std::ofstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save: %s", filename.c_str());
            return;
        }
        
        file << "x,y\n";
        for (const auto& point : path) {
            file << point.x << "," << point.y << "\n";
        }
        file.close();
        
        RCLCPP_INFO(this->get_logger(), "Saved: %s", filename.c_str());
    }

    // Save time-stamped trajectory to CSV file in current directory
    void save_trajectory_to_csv(const std::vector<TrajectoryPoint>& trajectory, 
                                 const std::string& filename)
    {
        std::ofstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save: %s", filename.c_str());
            return;
        }
        
        file << "x,y,time\n";
        for (const auto& point : trajectory) {
            file << point.x << "," << point.y << "," << point.time_stamp << "\n";
        }
        file.close();
        
        RCLCPP_INFO(this->get_logger(), "Saved: %s", filename.c_str());
    }

    // Member variables
    std::vector<TrajectoryPoint> trajectory_;
    std::shared_ptr<TrajectoryTrackerController> controller_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Pose current_pose_;
    bool is_initialized_;
    std::default_random_engine random_generator_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}