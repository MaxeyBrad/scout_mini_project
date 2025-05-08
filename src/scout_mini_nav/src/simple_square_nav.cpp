#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class SimpleSquareNav {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    
public:
    SimpleSquareNav() {
        // Use the correct topic
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ROS_INFO("Publishing to /cmd_vel");
    }
    
    void driveForward(double seconds, double speed = 0.2) {
        ROS_INFO("Driving forward for %.1f seconds", seconds);
        geometry_msgs::Twist cmd;
        cmd.linear.x = speed;
        cmd_vel_pub_.publish(cmd);
        ros::Duration(seconds).sleep();
    }
    
    void turn(double seconds, double speed = 0.5) {
        ROS_INFO("Turning for %.1f seconds", seconds);
        geometry_msgs::Twist cmd;
        cmd.angular.z = speed;
        cmd_vel_pub_.publish(cmd);
        ros::Duration(seconds).sleep();
    }
    
    void stop() {
        ROS_INFO("Stopping");
        geometry_msgs::Twist cmd;
        cmd_vel_pub_.publish(cmd);
        ros::Duration(1.0).sleep();
    }
    
    void runSquarePattern() {
        ROS_INFO("Running square pattern");
        
        for (int i = 0; i < 4; i++) {
            ROS_INFO("Side %d of square", i+1);
            driveForward(5.0);
            stop();
            turn(3.0);
            stop();
        }
        
        ROS_INFO("Square pattern complete");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_square_nav_node");
    
    // Create node handle first to initialize ROS time
    ros::NodeHandle nh;
    
    ROS_INFO("Simple Square Nav starting...");
    ros::Duration(5.0).sleep(); // Wait for simulator to initialize
    
    SimpleSquareNav navigator;
    navigator.runSquarePattern();
    
    return 0;
}
