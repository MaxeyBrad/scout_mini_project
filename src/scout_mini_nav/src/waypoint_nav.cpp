#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <cmath>

class SimpleNavigation {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    tf::TransformListener tf_listener_;
    
    double linear_speed_;
    double angular_speed_;
    int current_segment_;
    
    std::string base_frame_;
    std::string odom_frame_;
    
    // State machine states
    enum State {
        DRIVING_FORWARD,
        TURNING,
        STOPPED
    };
    State current_state_;
    ros::Time state_start_time_;
    
public:
    SimpleNavigation() 
        : linear_speed_(0.3), 
          angular_speed_(0.5), 
          current_segment_(0),
          current_state_(STOPPED),
          base_frame_("base_link"),
          odom_frame_("odom") {
        
        // Initialize command velocity publisher
        // Use the correct topic for Scout Mini
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/scout_mini/cmd_vel", 10);
        
        // Get parameters
        nh_.param("linear_speed", linear_speed_, 0.3);
        nh_.param("angular_speed", angular_speed_, 0.5);
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        
        ROS_INFO("Simple Navigation initialized with speeds: linear=%.2f, angular=%.2f", 
                linear_speed_, angular_speed_);
        ROS_INFO("Using frames: base_frame=%s, odom_frame=%s", 
                base_frame_.c_str(), odom_frame_.c_str());
    }
    
    void run() {
        ros::Rate rate(10);  // 10 Hz
        
        // Wait for everything to initialize
        ROS_INFO("Waiting 5 seconds before starting navigation...");
        ros::Duration(5.0).sleep();
        
        // Try to get the transform - check if it exists
        if (!waitForTransform()) {
            ROS_WARN("Could not get transform between %s and %s. Using dead reckoning navigation.",
                   odom_frame_.c_str(), base_frame_.c_str());
        }
        
        // Start with driving forward
        startNextSegment();
        
        while (ros::ok()) {
            updateNavigation();
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    bool waitForTransform() {
        int attempts = 0;
        const int max_attempts = 10;
        
        while (attempts < max_attempts && ros::ok()) {
            try {
                tf::StampedTransform transform;
                if (tf_listener_.waitForTransform(odom_frame_, base_frame_, ros::Time(0), ros::Duration(1.0))) {
                    tf_listener_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), transform);
                    ROS_INFO("Transform from %s to %s is available.", 
                           odom_frame_.c_str(), base_frame_.c_str());
                    return true;
                }
            } catch (tf::TransformException &ex) {
                ROS_WARN("TF exception: %s", ex.what());
            }
            
            attempts++;
            ROS_WARN("Waiting for transform between %s and %s (attempt %d/%d)",
                   odom_frame_.c_str(), base_frame_.c_str(), attempts, max_attempts);
            ros::Duration(1.0).sleep();
        }
        
        return false;
    }
    
    void startNextSegment() {
        if (current_segment_ >= 4) {
            // We've completed the square
            current_state_ = STOPPED;
            ROS_INFO("Navigation complete - square pattern finished");
            return;
        }
        
        if (current_segment_ % 2 == 0) {
            // Even segments drive forward
            current_state_ = DRIVING_FORWARD;
            ROS_INFO("Starting forward segment %d", current_segment_);
        } else {
            // Odd segments turn
            current_state_ = TURNING;
            ROS_INFO("Starting turning segment %d", current_segment_);
        }
        
        // Record start time for this segment
        state_start_time_ = ros::Time::now();
    }
    
    void updateNavigation() {
        geometry_msgs::Twist cmd_vel;
        
        switch (current_state_) {
            case DRIVING_FORWARD:
                // Drive forward for 3 seconds
                if ((ros::Time::now() - state_start_time_).toSec() < 3.0) {
                    cmd_vel.linear.x = linear_speed_;
                    cmd_vel.angular.z = 0.0;
                } else {
                    // Start the next segment (turning)
                    current_segment_++;
                    startNextSegment();
                }
                break;
                
            case TURNING:
                // Turn 90 degrees (roughly 3.14/2 radians)
                // Turn duration = angle / angular_speed
                if ((ros::Time::now() - state_start_time_).toSec() < (M_PI/2.0)/angular_speed_) {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = angular_speed_;
                } else {
                    // Start the next segment (forward)
                    current_segment_++;
                    startNextSegment();
                }
                break;
                
            case STOPPED:
                // Do nothing - keep sending zero velocity
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                break;
        }
        
        // Publish the command
        cmd_vel_pub_.publish(cmd_vel);
        
        // Try to get current position for debugging
        try {
            tf::StampedTransform transform;
            tf_listener_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), transform);
            
            // Print current position to help debug
            ROS_INFO("Robot position: (%.2f, %.2f, %.2f)",
                   transform.getOrigin().x(),
                   transform.getOrigin().y(),
                   transform.getOrigin().z());
        } catch (tf::TransformException &ex) {
            // Don't log every time
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_nav_node");
    
    ROS_INFO("Starting Simple Navigation Node");
    
    SimpleNavigation navigator;
    navigator.run();
    
    return 0;
}
