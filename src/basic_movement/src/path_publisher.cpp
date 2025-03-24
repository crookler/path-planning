#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class PathPublisher : public rclcpp::Node {
public:
    PathPublisher() : Node("dummy_path_publisher") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PathPublisher::publish_dummy_path, this));
    } 
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_dummy_path() {
        auto msg = nav_msgs::msg::Path();
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        msg.header.stamp = rclcpp::Clock().now();
        msg.header.frame_id = "map";

        for (int i = 0; i < 10; i++) {
            geometry_msgs::msg::PoseStamped currentPose;
            currentPose.header.stamp = rclcpp::Clock().now();
            currentPose.header.frame_id = "map";
            
            // Coordinates relative to the map frame of reference
            currentPose.pose.position.x = i;
            currentPose.pose.position.y = 0;
            currentPose.pose.position.z = 0;

            // Quaternion representation (magnitude 1 so no rotation along x y or z)
            currentPose.pose.orientation.w = 1;
            poses.push_back(currentPose);
        }

        msg.poses = poses;
        RCLCPP_INFO(this->get_logger(), "Publishing %ld data points", msg.poses.size());
        path_pub_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}


