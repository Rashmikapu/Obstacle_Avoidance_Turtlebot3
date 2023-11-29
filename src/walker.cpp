
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Using literals to the sensor messages types for more readability
using std::placeholders::_1;
using namespace std::chrono_literals;


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
    
    void publish_velocity(auto x_vel, auto z_vel) {
        auto vel = geometry_msgs::msg::Twist();
        vel.linear.x = x_vel;
        vel.angular.z = -z_vel;
        velocity_pub->publish(vel);
    }

    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AvoidObstacles>());
    rclcpp::shutdown();
    return 0;
}