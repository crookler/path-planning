#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "path_interfaces/msg/flat_grid.hpp"
#include "path_interfaces/action/a_search_query.hpp"
#include "path_interfaces/msg/path_query.hpp"

namespace environment_mapping {

class AStarSearch : public rclcpp::Node {
public:
    using ASearchQuery = path_interfaces::action::ASearchQuery;
    using GoalHandlePath = rclcpp_action::ServerGoalHandle<ASearchQuery>;

    explicit AStarSearch(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("a_star_action_server", options) {
        this->action_server = rclcpp_action::create_server<ASearchQuery>(
            this,
            "a_star_search",
            std::bind(&AStarSearch::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AStarSearch::handle_cancel, this, std::placeholders::_1),
            std::bind(&AStarSearch::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<ASearchQuery>::SharedPtr action_server;
    rclcpp::Subscription<path_interfaces::msg::FlatGrid>::SharedPtr reduced_grid_subscriber;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuiud, 
        std::shared_ptr<const ASearchQuery::Goal> goal) {
            const std::string& frame = goal->query.goal.header.frame_id; // Read only
            RCLCPP_INFO(this->get_logger(), "Received goal with frame_id: %s", frame.c_str());
            
            // Only accept goal that is set in the global frame (map)
            if (frame != "map") {
                RCLCPP_WARN(this->get_logger(), "Rejected goal due to invalid frame_id: %s", frame.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePath> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Cancel goal");
            return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<GoalHandlePath> goal_handle) {
            // Allow thread to run in parallel
            std::thread{std::bind(&AStarSearch::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(
        const std::shared_ptr<GoalHandlePath> goal_handle) {
        // A C++ Program to implement A* Search Algorithm https://www.geeksforgeeks.org/a-search-algorithm/
    }

};

}