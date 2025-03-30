#include "rclcpp/rclcpp.hpp"
#include "path_interfaces/action/path_query.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace environment_mapping {

class AStarSearch : public rclcpp::Node {
public:
    explicit AStarSearch(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("a_star_action_server", options) {
        this->action_server = rclcpp_action::create_server<PathQuery>(
            this,
            "fibonacci",
            std::bind(&AStarSearch::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AStarSearch::handle_cancel, this, std::placeholders::_1),
            std::bind(&AStarSearch::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<PathQuery>::SharedPtr action_server;

    // TODO:
    rclcpp_action::GoalResponse handle_goal();
    rclcpp_action::GoalResponse handle_cancel();
    rclcpp_action::GoalResponse handle_accepted();
};

}