#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <vector>

class BasicWaypointFollower {
private:
   ros::NodeHandle nh_;
   ros::Publisher cmd_vel_pub_;
   ros::Subscriber odom_sub_;
   tf::TransformListener tf_listener_;
   
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
   ros::Time last_cmd_time_;
   bool using_dead_reckoning_;
   
public:
   BasicWaypointFollower() : 
       current_waypoint_(0),
       robot_x_(0.0),
       robot_y_(0.0),
       robot_yaw_(0.0),
       position_initialized_(false),
       using_dead_reckoning_(true) {
       
       // Get parameters
       nh_.param("linear_speed", linear_speed_, 0.2);  // Slower for safety
       nh_.param("angular_speed", angular_speed_, 0.5);
       nh_.param("waypoint_reached_distance", waypoint_reached_distance_, 0.3);
       nh_.param("angle_tolerance", angle_tolerance_, 0.1);
       
       // Define waypoints for a square pattern
       waypoints_.push_back(std::make_pair(0.0, 0.0));   // Start
       waypoints_.push_back(std::make_pair(5.0, 0.0));   // Forward 2m
       waypoints_.push_back(std::make_pair(5.0, 5.0));   // Left 2m
       waypoints_.push_back(std::make_pair(0.0, 5.0));   // Back 2m
       waypoints_.push_back(std::make_pair(0.0, 0.0));   // Right 2m back to start
       
       // Initialize ROS communication
       cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
       
       // Try to use odometry if available
       try {
           odom_sub_ = nh_.subscribe("/odometry/filtered", 10, &BasicWaypointFollower::odomCallback, this);
           ROS_INFO("Subscribed to /odometry/filtered");
       } catch (...) {
           ROS_WARN("Failed to subscribe to odometry, using dead reckoning");
           using_dead_reckoning_ = true;
       }
       
       last_cmd_time_ = ros::Time::now();
       
       ROS_INFO("Waypoint Follower initialized with %zu waypoints", waypoints_.size());
       ROS_INFO("Navigation parameters: linear_speed=%.2f, angular_speed=%.2f", 
               linear_speed_, angular_speed_);
       
       // Wait for simulator to initialize
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
       ROS_INFO_THROTTLE(5.0, "Received odometry: position=(%.2f, %.2f), yaw=%.2f", 
               robot_x_, robot_y_, robot_yaw_);
   }
   
   void updatePoseDeadReckoning() {
       // Use smaller increments
       double pos_step = 0.02;  // 2cm per update
       double angle_step = 0.03; // ~1.7 degrees per update
       
       // Get current waypoint coordinates
       double target_x = waypoints_[current_waypoint_].first;
       double target_y = waypoints_[current_waypoint_].second;
       
       // Calculate direction vector to waypoint
       double dx = target_x - robot_x_;
       double dy = target_y - robot_y_;
       double target_yaw = atan2(dy, dx);
       
       // Normalize angle difference
       double angle_diff = target_yaw - robot_yaw_;
       while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
       while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
       
       // Decide whether to turn or move forward
       if (std::abs(angle_diff) > angle_tolerance_) {
           // We're turning - update yaw only
           robot_yaw_ += (angle_diff > 0) ? angle_step : -angle_step;
       } else {
           // We're moving forward - update position
           robot_x_ += pos_step * cos(robot_yaw_);
           robot_y_ += pos_step * sin(robot_yaw_);
       }
       
       // Normalize yaw
       while (robot_yaw_ > M_PI) robot_yaw_ -= 2.0 * M_PI;
       while (robot_yaw_ < -M_PI) robot_yaw_ += 2.0 * M_PI;
   }
   
   void moveToCurrentWaypoint() {
       // Make sure we haven't reached the end
       if (current_waypoint_ >= waypoints_.size()) {
           stopRobot();
           ROS_INFO("Navigation complete - all waypoints reached!");
           return;
       }
       
       // Make sure we have position data
       if (!position_initialized_) {
           ROS_WARN_THROTTLE(1.0, "Position not initialized, assuming at origin");
           robot_x_ = 0.0;
           robot_y_ = 0.0;
           robot_yaw_ = 0.0;
           position_initialized_ = true;
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
       
       // Always update position using dead reckoning
       updatePoseDeadReckoning();
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
           // Move toward current waypoint
           moveToCurrentWaypoint();
           
           ros::spinOnce();
           rate.sleep();
       }
   }
};

int main(int argc, char** argv) {
   ros::init(argc, argv, "basic_waypoint_follower_node");
   ros::NodeHandle nh;  // Create NodeHandle immediately to initialize time
   
   ROS_INFO("Basic waypoint follower node starting...");
   
   BasicWaypointFollower follower;
   follower.run();
   
   return 0;
}
