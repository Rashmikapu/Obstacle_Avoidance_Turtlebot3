/**
 * @file walker.cpp
 * @author Rashmi Kapu (rashmik@umd.edu)
 * @brief This node implements a class which creates a subscriber to the Lidar on Turtlebot4 burger
 and publisher to turtlebot3 burger. Based on the Lidar point cloud data, the node publishes 
 velocities and orientations to the turtlebot accordingly, in order to avoid obstacles. 
 * @version 0.1
 * @date 2023-11-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Using literals to the sensor messages types for more readability
using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief Class AvoidObstacles initialises publisher and subscriber nodes and
 binds lidar_callback to the subscriber node
 * 
 */
class AvoidObstacles : public rclcpp::Node {
 public:
 
    AvoidObstacles() :
        Node("walker") {
            // Initialize the publisher and subscriber
            lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>
                                        ("scan", 10, std::bind(&AvoidObstacles::lidar_callback, this, _1));
            velocity_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        }

 private:
 /**
  * @brief Reads lidar data and implements obstacle avoidance. Calls function
  publish_velocity to publish to cmd_vel topic.
  * 
  * @param msg 
  */
    void lidar_callback(const sensor_msgs::msg::LaserScan& msg) {
        
        if (msg.header.stamp.sec == 0) {
            return;
        }
        auto scan_data = msg.ranges;

        // Setting field of view = -50 to +50 degrees
        int min_angle = 310;
        int max_angle = 50;
        auto threshold = 0.5;
        for (int i = min_angle; i < min_angle + max_angle; i++) {
            if (scan_data[i % 360] < threshold) {
                // turn
                publish_velocity(0.0, 0.1);
            } else {
                // move forward
                publish_velocity(0.075, 0.0);
            }
        }
    }
    
    /**
     * @brief Publishes linear and angular velocities on cmd_vel topic to
     control the motion of turtlebot3 burger.
     * 
     * @param x_vel 
     * @param z_vel 
     */
    void publish_velocity(auto x_vel, auto z_vel) {
        auto vel = geometry_msgs::msg::Twist();
        vel.linear.x = x_vel;
        vel.angular.z = -z_vel;
        velocity_pub->publish(vel);
    }

    // Create pointers to subscriber and publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;
};

/**
 * @brief Creates a spin to call AvoidObstacles class to implement walker node.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AvoidObstacles>());
    rclcpp::shutdown();
    return 0;
}