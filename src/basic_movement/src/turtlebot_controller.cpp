#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class TurtleBotController : public rclcpp::Node {
public:
    TurtleBotController() : Node("turtlebot_controller") {
        // Publisher for velocity commands (/cmd_vel is the turtlebot velocity topic)
        velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber for Lidar scan data (scan_callback is callback function bound to this instance with one )
        scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TurtleBotController::scan_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;

    // Passing message from /scan topic
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Publish twist to /cmd_vel (linear and angular velocities)
        auto cmd = geometry_msgs::msg::Twist();
        float min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());

        if (min_distance < 0.5) { // Stop and turn if too close to an obstacle
            cmd.linear.x = 0.0;   // Stop forward motion
            cmd.angular.z = 1.0;  // Rotate
        } else {
            cmd.linear.x = 0.2;   // Move forward
            cmd.angular.z = 0.0;  // No rotation
        }

        velocity_publisher->publish(cmd);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleBotController>());
    rclcpp::shutdown();
    return 0;
}
