#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <vector>

class SimpleNavigation {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    
    // Parameters
    double linear_speed_;
    double angular_speed_;
    
    enum State {
        FORWARD,
        TURN,
        STOP
    };
    
    State current_state_;
    int segment_counter_;
    ros::Time state_start_time_;

public:
    SimpleNavigation() : current_state_(FORWARD), segment_counter_(0) {
        // Initialize ROS node
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        // Get parameters from parameter server or use defaults
        nh_.param("linear_speed", linear_speed_, 0.3);
        nh_.param("angular_speed", angular_speed_, 0.5);
        
        state_start_time_ = ros::Time::now();
        
        ROS_INFO("Simple navigation initialized");
    }
    
    void run() {
        ros::Rate rate(10); // 10 Hz
        
        ROS_INFO("Starting simple navigation");
        
        // Allow time for simulation to fully initialize
        ros::Duration(5.0).sleep();
        
        while (ros::ok()) {
            switch (current_state_) {
                case FORWARD:
                    driveForward();
                    break;
                case TURN:
                    turn();
                    break;
                case STOP:
                    stopRobot();
                    break;
            }
            
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    void driveForward() {
        // Drive forward for 3 seconds
        ros::Duration elapsed = ros::Time::now() - state_start_time_;
        
        if (elapsed.toSec() < (3.0 + segment_counter_ * 0.5)) {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = linear_speed_;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_.publish(cmd_vel);
            
            ROS_INFO_THROTTLE(1.0, "Driving forward: %.1f seconds", elapsed.toSec());
        } else {
            // Switch to turning
            current_state_ = TURN;
            state_start_time_ = ros::Time::now();
            ROS_INFO("Switching to turning state");
        }
    }
    
    void turn() {
        // Turn approximately 90 degrees
        ros::Duration elapsed = ros::Time::now() - state_start_time_;
        
        if (elapsed.toSec() < 3.14 / (2 * angular_speed_)) {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = angular_speed_;
            cmd_vel_pub_.publish(cmd_vel);
            
            ROS_INFO_THROTTLE(1.0, "Turning: %.1f seconds", elapsed.toSec());
        } else {
            // Switch to forward or stop
            segment_counter_++;
            
            if (segment_counter_ < 4) {
                current_state_ = FORWARD;
                state_start_time_ = ros::Time::now();
                ROS_INFO("Switching to forward state, segment %d", segment_counter_);
            } else {
                current_state_ = STOP;
                ROS_INFO("Navigation complete");
            }
        }
    }
    
    void stopRobot() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_nav_node");
    
    ROS_INFO("Simple navigation node starting...");
    
    SimpleNavigation navigator;
    navigator.run();
    
    return 0;
}
