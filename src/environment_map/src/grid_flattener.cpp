#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "path_interfaces/msg/flat_grid.hpp"

class GridFlattener : public rclcpp::Node {
public:
    GridFlattener() : Node("grid_flattener") {
        // Read value of occupancy grid from SLAM topic
        grid_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 
            10, std::bind(&GridFlattener::grid_callback, this, std::placeholders::_1));
    
        // Broadcast reduced occupancy grid
        flattened_publisher = this->create_publisher<path_interfaces::msg::FlatGrid>("/flat_grid", 10);
    }   

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_subscriber;
    rclcpp::Publisher<path_interfaces::msg::FlatGrid>::SharedPtr flattened_publisher;

    // Pass occupancy grid by reference (not sure how big this is)
    void grid_callback(const nav_msgs::msg::OccupancyGrid & occupancy_grid) {
        // Initialize flat grid (assign height and width from occupancy grid and then just read boolean values in)
        // More useful than raw percentages for search algorithms like BFS (might just use occupancy_grid directly for A*/Dijkstra's)
        path_interfaces::msg::FlatGrid flat_grid;
        flat_grid.height = occupancy_grid.info.height;
        flat_grid.width = occupancy_grid.info.width;

        // occupied_squares gets converted to std::vector<bool> so just call resize
        flat_grid.occupied_squares.resize(flat_grid.height * flat_grid.width, false);

        // Just construct reduced grid based on probability threshold (simple binary occupied vector)
        const int probability_threshold = 50;

        for (size_t i = 0; i < occupancy_grid.data.size(); i++) {
            flat_grid.occupied_squares[i] = occupancy_grid.data[i] > probability_threshold;
        }
        
        flattened_publisher->publish(flat_grid);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridFlattener>());
  rclcpp::shutdown();
  return 0;
}