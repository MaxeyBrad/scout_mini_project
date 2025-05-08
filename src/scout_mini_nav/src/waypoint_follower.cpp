#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <vector>

class WaypointFollower {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    
    // Waypoints (x, y) in meters
    std::vector<std::pair<double, double>> waypoints_;
    int current_waypoint_;
    
    // Navigation parameters
    double linear_speed_;
    double angular_speed_;
    double waypoint_reached_distance_;
    double angle_tolerance_;
    
    // Position tracking
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    bool position_initialized_;
    
public:
    WaypointFollower() : 
        current_waypoint_(0),
        robot_x_(0.0),
        robot_y_(0.0),
        robot_yaw_(0.0),
        position_initialized_(false) {
        
        // Initialize ROS communication
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        odom_sub_ = nh_.subscribe("/odometry/filtered", 10, &WaypointFollower::odomCallback, this);
        
        // Get parameters
        nh_.param("linear_speed", linear_speed_, 0.3);
        nh_.param("angular_speed", angular_speed_, 0.5);
        nh_.param("waypoint_reached_distance", waypoint_reached_distance_, 0.3);
        nh_.param("angle_tolerance", angle_tolerance_, 0.1);
        
        // Define agricultural pattern waypoints
        // Simple row pattern (customize as needed)
        waypoints_.push_back(std::make_pair(0.0, 0.0));   // Start position
        waypoints_.push_back(std::make_pair(2.0, 0.0));   // End of row 1
        waypoints_.push_back(std::make_pair(2.0, 1.0));   // Move up
        waypoints_.push_back(std::make_pair(0.0, 1.0));   // End of row 2
        waypoints_.push_back(std::make_pair(0.0, 2.0));   // Move up
        waypoints_.push_back(std::make_pair(2.0, 2.0));   // End of row 3
        waypoints_.push_back(std::make_pair(2.0, 3.0));   // Move up
        waypoints_.push_back(std::make_pair(0.0, 3.0));   // End of row 4
        waypoints_.push_back(std::make_pair(0.0, 0.0));   // Return to start
        
        ROS_INFO("Waypoint Follower initialized with %zu waypoints", waypoints_.size());
        ROS_INFO("Navigation parameters: linear_speed=%.2f, angular_speed=%.2f", 
                linear_speed_, angular_speed_);
        
        // Wait for simulation to fully initialize
        ros::Duration(5.0).sleep();
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Get position from odometry message
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        
        // Convert quaternion to Euler angles
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, robot_yaw_);
        
        position_initialized_ = true;
    }
    
    bool updateRobotPose() {
        // Now we're just using the odometry callback
        return position_initialized_;
    }
    
    void moveToCurrentWaypoint() {
        // Make sure we haven't reached the end
        if (current_waypoint_ >= waypoints_.size()) {
            stopRobot();
            ROS_INFO("Navigation complete - all waypoints reached!");
            return;
        }
        
        // Wait for position data to be initialized
        if (!position_initialized_) {
            ROS_WARN_THROTTLE(1.0, "Waiting for odometry data...");
            return;
        }
        
        // Get current waypoint coordinates
        double target_x = waypoints_[current_waypoint_].first;
        double target_y = waypoints_[current_waypoint_].second;
        
        // Calculate distance to waypoint
        double dx = target_x - robot_x_;
        double dy = target_y - robot_y_;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Debug output
        ROS_INFO_THROTTLE(2.0, "Robot at (%.2f, %.2f, %.2f), waypoint %d at (%.2f, %.2f), distance: %.2f", 
                 robot_x_, robot_y_, robot_yaw_, current_waypoint_, target_x, target_y, distance);
        
        // Check if we've reached the waypoint
        if (distance < waypoint_reached_distance_) {
            ROS_INFO("Reached waypoint %d at (%.2f, %.2f)", current_waypoint_, target_x, target_y);
            current_waypoint_++;
            
            // Pause briefly at the waypoint
            stopRobot();
            ros::Duration(1.0).sleep();
            
            return;
        }
        
        // Calculate angle to waypoint
        double target_yaw = std::atan2(dy, dx);
        
        // Calculate the difference between our current heading and the target heading
        double angle_diff = target_yaw - robot_yaw_;
        
        // Normalize to range [-PI, PI]
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        
        // Create velocity command
        geometry_msgs::Twist cmd_vel;
        
        // If we're not facing the right direction, turn first
        if (std::abs(angle_diff) > angle_tolerance_) {
            // Turn toward waypoint
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = (angle_diff > 0) ? angular_speed_ : -angular_speed_;
            
            ROS_INFO_THROTTLE(1.0, "Turning to face waypoint, angle diff: %.2f", angle_diff);
        } else {
            // We're facing the right direction, move forward
            cmd_vel.linear.x = linear_speed_;
            
            // Small angular correction while moving to stay on course
            cmd_vel.angular.z = 0.5 * angle_diff;
            
            ROS_INFO_THROTTLE(1.0, "Moving toward waypoint");
        }
        
        // Send the command to the robot
        cmd_vel_pub_.publish(cmd_vel);
    }
    
    void stopRobot() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
    }
    
    void run() {
        ros::Rate rate(10); // 10 Hz control loop
        
        ROS_INFO("Starting waypoint navigation to first waypoint at (%.2f, %.2f)",
                waypoints_[current_waypoint_].first, waypoints_[current_waypoint_].second);
        
        while (ros::ok()) {
            // Update robot's position
            updateRobotPose();
            
            // Move toward current waypoint
            moveToCurrentWaypoint();
            
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_follower_node");
    
    ROS_INFO("Waypoint follower node starting...");
    
    WaypointFollower follower;
    follower.run();
    
    return 0;
}
