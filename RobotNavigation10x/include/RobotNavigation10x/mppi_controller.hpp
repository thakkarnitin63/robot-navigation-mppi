#ifndef MPPI_CONTROLLER_HPP_
#define MPPI_CONTROLLER_HPP_

#include <vector>
#include <random>
#include <memory>
#include "RobotNavigation10x/types.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * Static obstacle representation from simulation environment.
 * Supports both box and cylindrical obstacles with arbitrary rotation.
 */
struct StaticObstacle
{
    enum Type { BOX, CYLINDER };
    
    Type type;                    // Obstacle geometry type
    double x, y;                  // Center position in world frame (meters)
    double size_x, size_y;        // Box dimensions (meters)
    double radius;                // Cylinder radius (meters)
    double rotation;              // Box rotation angle (radians)
};

/**
 * Model Predictive Path Integral (MPPI) controller with obstacle avoidance.
 * Sampling-based stochastic optimal control using importance sampling.
 * Combines static obstacle knowledge with real-time LiDAR sensing.
 */
class MPPIController
{
public:
    /**
     * MPPI algorithm configuration parameters.
     */
    struct MPPIParams
    {
        // Sampling configuration
        int num_samples;              // Number of trajectory samples (K)
        int horizon_steps;            // Prediction horizon length (T)
        double dt;                    // Time step for simulation (seconds)
        
        // Robot constraints
        double max_linear_vel;        // Maximum forward velocity (m/s)
        double max_angular_vel;       // Maximum rotation rate (rad/s)
        double max_linear_accel;      // Linear acceleration limit (m/s²)
        double max_angular_accel;     // Angular acceleration limit (rad/s²)
        
        // Exploration noise parameters
        double linear_vel_noise_std;  // Standard deviation for velocity sampling
        double angular_vel_noise_std; // Standard deviation for angular velocity
        
        // Cost function configuration
        double lambda;                // Temperature parameter (inverse)
        double weight_tracking;       // Path following weight
        double weight_static_obstacle;  // Known obstacle avoidance weight
        double weight_lidar_obstacle;   // Dynamic obstacle avoidance weight
        double weight_control;        // Control effort weight
        double weight_smoothness;     // Motion smoothness weight
        double weight_goal;           // Goal reaching weight
        
        // Safety parameters
        double robot_radius;          // Robot footprint radius (meters)
        double safe_distance;         // Minimum safe clearance (meters)
        double danger_distance;       // Distance to start penalizing (meters)
        
        // Default parameter values
        MPPIParams()
            : num_samples(1000),
              horizon_steps(30),
              dt(0.05),
              max_linear_vel(0.22),
              max_angular_vel(2.84),
              max_linear_accel(0.5),
              max_angular_accel(3.0),
              linear_vel_noise_std(0.1),
              angular_vel_noise_std(0.5),
              lambda(1.0),
              weight_tracking(10.0),
              weight_static_obstacle(150.0),
              weight_lidar_obstacle(100.0),
              weight_control(0.1),
              weight_smoothness(1.0),
              weight_goal(50.0),
              robot_radius(0.105),
              safe_distance(0.3),
              danger_distance(0.6)
        {}
    };
    
    /**
     * Robot state representation for trajectory prediction.
     */
    struct RobotState
    {
        double x, y, theta;    // Position and orientation
        double v, omega;       // Linear and angular velocities
    };
    
    /**
     * Control input for differential drive robot.
     */
    struct ControlInput
    {
        double v, omega;       // Velocity commands
    };

    /**
     * Initialize MPPI controller with given parameters.
     */
    MPPIController(const MPPIParams& params = MPPIParams());
    
    /**
     * Load static obstacles from the environment.
     * Should be called once during initialization.
     */
    void set_static_obstacles(const std::vector<StaticObstacle>& obstacles);
    
    /**
     * Compute optimal control using MPPI algorithm.
     * Samples trajectories, evaluates costs, and returns weighted optimal control.
     * 
     * @param current_pose Robot's current pose
     * @param current_velocity Robot's current velocity
     * @param trajectory Reference trajectory to follow
     * @param laser_scan LiDAR data for dynamic obstacle detection (optional)
     * @return Optimal velocity commands
     */
    geometry_msgs::msg::Twist compute_control(
        const geometry_msgs::msg::Pose& current_pose,
        const geometry_msgs::msg::Twist& current_velocity,
        const std::vector<TrajectoryPoint>& trajectory,
        const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan = nullptr);
    
    /**
     * Check if robot has reached the goal position.
     */
    bool is_goal_reached(
        const geometry_msgs::msg::Pose& current_pose,
        const std::vector<TrajectoryPoint>& trajectory);
    
    /**
     * Get the best predicted trajectory for visualization.
     */
    std::vector<RobotState> get_predicted_trajectory() const {
        return best_trajectory_;
    }
    
    /**
     * Get static obstacles for visualization.
     */
    std::vector<StaticObstacle> get_static_obstacles() const {
        return static_obstacles_;
    }

private:
    MPPIParams params_;
    std::default_random_engine random_generator_;
    std::normal_distribution<double> noise_distribution_;
    
    std::vector<ControlInput> prev_control_sequence_;  // Warm start
    std::vector<RobotState> best_trajectory_;          // For visualization
    
    std::vector<StaticObstacle> static_obstacles_;     // Known obstacles
    std::vector<Point2D> lidar_obstacles_;             // Sensed obstacles
    
    // State conversion and simulation
    RobotState pose_to_state(const geometry_msgs::msg::Pose& pose,
                             const geometry_msgs::msg::Twist& velocity);
    RobotState simulate_step(const RobotState& state, const ControlInput& control, double dt);
    
    // MPPI sampling
    std::vector<ControlInput> generate_control_sequence(const std::vector<ControlInput>& base);
    std::vector<RobotState> rollout_trajectory(const RobotState& initial, 
                                                 const std::vector<ControlInput>& controls);
    
    // Cost computation
    double compute_trajectory_cost(const std::vector<RobotState>& trajectory,
                                   const std::vector<ControlInput>& controls,
                                   const std::vector<TrajectoryPoint>& reference,
                                   const RobotState& current);
    double compute_tracking_cost(const std::vector<RobotState>& trajectory,
                                 const std::vector<TrajectoryPoint>& reference);
    double compute_static_obstacle_cost(const std::vector<RobotState>& trajectory);
    double compute_lidar_obstacle_cost(const std::vector<RobotState>& trajectory);
    double compute_control_cost(const std::vector<ControlInput>& controls);
    double compute_smoothness_cost(const std::vector<ControlInput>& controls);
    double compute_goal_cost(const RobotState& final, 
                             const std::vector<TrajectoryPoint>& reference);
    
    // Obstacle detection
    size_t find_closest_trajectory_point(const RobotState& state,
                                         const std::vector<TrajectoryPoint>& trajectory);
    void process_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                            const RobotState& current);
    bool is_colliding_with_box(double x, double y, const StaticObstacle& box);
    bool is_colliding_with_cylinder(double x, double y, const StaticObstacle& cylinder);
    double distance_to_static_obstacle(double x, double y);
    double distance_to_lidar_obstacle(double x, double y);
    
    // Utility functions
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q);
    double normalize_angle(double angle);
    
    template<typename T>
    T clamp(T value, T min_val, T max_val) {
        return std::max(min_val, std::min(value, max_val));
    }
};

#endif // MPPI_CONTROLLER_HPP_